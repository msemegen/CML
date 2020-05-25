/*
    Name: I2C.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <hal/stm32l452xx/I2C.hpp>

//cml
#include <hal/core/systick.hpp>
#include <hal/stm32l452xx/mcu.hpp>
#include <utils/wait.hpp>

namespace {

using namespace cml::common;
using namespace cml::hal::stm32l452xx;

void i2c_1_enable(uint32 a_clock_source, uint32 a_irq_priority)
{
    assert(0 != a_clock_source);

    set_flag(&(RCC->CCIPR), RCC_CCIPR_I2C1SEL, a_clock_source);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN);

    NVIC_SetPriority(I2C1_EV_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
}

void i2c_1_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN);
    NVIC_DisableIRQ(I2C1_EV_IRQn);
}

void i2c_2_enable(uint32 a_clock_source, uint32 a_irq_priority)
{
    assert(0 != a_clock_source);

    set_flag(&(RCC->CCIPR), RCC_CCIPR_I2C2SEL, a_clock_source);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C2EN);

    NVIC_SetPriority(I2C2_EV_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C2_EV_IRQn);
}

void i2c_2_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C2EN);
    NVIC_DisableIRQ(I2C2_EV_IRQn);
}

void i2c_3_enable(uint32 a_clock_source, uint32 a_irq_priority)
{
    assert(0 != a_clock_source);

    set_flag(&(RCC->CCIPR), RCC_CCIPR_I2C3SEL, a_clock_source);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C3EN);

    NVIC_SetPriority(I2C3_EV_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C3_EV_IRQn);
}

void i2c_3_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C3EN);
    NVIC_DisableIRQ(I2C3_EV_IRQn);
}

void i2c_4_enable(uint32 a_clock_source, uint32 a_irq_priority)
{
    assert(0 != a_clock_source);

    set_flag(&(RCC->CCIPR2), RCC_CCIPR2_I2C4SEL, a_clock_source);
    set_flag(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN);

    NVIC_SetPriority(I2C4_EV_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C4_EV_IRQn);
}

void i2c_4_disable()
{
    clear_flag(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN);
    NVIC_DisableIRQ(I2C4_EV_IRQn);
}

enum class Transfer_type
{
    transmit,
    receive
};


struct Controller
{
    using Enable_function  = void(*)(uint32 a_clock_source, uint32 a_irq_priority);
    using Disable_function = void(*)();

    I2C_TypeDef* p_registers        = nullptr;
    I2C_master* p_i2c_master_handle = nullptr;
    I2C_slave* p_i2c_slave_handle   = nullptr;

    Enable_function enable   = nullptr;
    Disable_function disable = nullptr;
};

Controller controllers[]
{
    { I2C1, nullptr, nullptr, i2c_1_enable, i2c_1_disable },
    { I2C2, nullptr, nullptr, i2c_2_enable, i2c_2_disable },
    { I2C3, nullptr, nullptr, i2c_3_enable, i2c_3_disable },
    { I2C4, nullptr, nullptr, i2c_4_enable, i2c_4_disable }
};

bool is_I2C_ISR_error(const I2C_TypeDef* a_p_i2c)
{
    return is_any_bit(a_p_i2c->ISR, I2C_ISR_TIMEOUT |
                                    I2C_ISR_PECERR  |
                                    I2C_ISR_OVR     |
                                    I2C_ISR_ARLO    |
                                    I2C_ISR_BERR    |
                                    I2C_ISR_NACKF);
}

void clear_I2C_ISR_errors(I2C_TypeDef* a_p_i2c)
{
    set_flag(&(a_p_i2c->ICR), I2C_ICR_TIMOUTCF |
                              I2C_ICR_PECCF    |
                              I2C_ICR_OVRCF    |
                              I2C_ICR_ARLOCF   |
                              I2C_ICR_BERRCF   |
                              I2C_ICR_NACKCF);
}


I2C_base::Bus_status get_bus_status_from_I2C_ISR(const I2C_TypeDef* a_p_i2c, Transfer_type a_transfer_type)
{
    if (true == is_flag(a_p_i2c->ISR, I2C_ISR_TIMEOUT))
    {
        return I2C_base::Bus_status::timeout;
    }

    if (true == is_flag(a_p_i2c->ISR, I2C_ISR_OVR))
    {
        switch (a_transfer_type)
        {
            case Transfer_type::receive:
                return I2C_base::Bus_status::overrun;

            case Transfer_type::transmit:
                return I2C_base::Bus_status::underrun;
        }
    }

    if (true == is_flag(a_p_i2c->ISR, I2C_ISR_ARLO))
    {
        return I2C_base::Bus_status::arbitration_lost;
    }

    if (true == is_flag(a_p_i2c->ISR, I2C_ISR_BERR))
    {
        return I2C_base::Bus_status::misplaced;
    }

    if (true == is_flag(a_p_i2c->ISR, I2C_ISR_NACKF))
    {
        return I2C_base::Bus_status::nack;
    }

    return I2C_base::Bus_status::ok;
}

I2C_base::Clock_source get_clock_source_from_RCC_CCIPR(I2C_base::Id a_id)
{
    switch (a_id)
    {
        case I2C_base::Id::_1:
        case I2C_base::Id::_2:
        case I2C_base::Id::_3:
        {
            return static_cast<I2C_base::Clock_source>(get_flag(RCC->CCIPR,
                                                                0x3 << (RCC_CCIPR_I2C1SEL_Pos + static_cast<uint32>(a_id) * 2)) >> RCC_CCIPR_I2C1SEL_Pos);
        }
        break;

        default:
        {
            return static_cast<I2C_base::Clock_source>(get_flag(RCC->CCIPR2, RCC_CCIPR2_I2C4SEL));
        }
        break;
    }
}

uint32 get_RCC_CCIPR_from_clock_source(I2C_base::Clock_source a_clock_source, I2C_base::Id a_i2c_id)
{
    switch (a_i2c_id)
    {
        case I2C_base::Id::_1:
        case I2C_base::Id::_2:
        case I2C_base::Id::_3:
        {
            return static_cast<uint32>(a_clock_source) << (RCC_CCIPR_I2C1SEL_Pos + static_cast<uint32>(a_i2c_id) * 2);
        }
        break;

        case I2C_base::Id::_4:
        {
            return static_cast<uint32>(a_clock_source);
        }
        break;
    }

    return 0;
}

} // namespace ::

extern "C"
{

void interupt_handler(uint32 a_controller_index)
{
    assert(nullptr != controllers[a_controller_index].p_i2c_master_handle ||
           nullptr != controllers[a_controller_index].p_i2c_slave_handle);

    if (nullptr != controllers[a_controller_index].p_i2c_master_handle)
    {
        i2c_handle_interrupt(controllers[a_controller_index].p_i2c_master_handle);
    }
    else if (nullptr != controllers[a_controller_index].p_i2c_slave_handle)
    {
        i2c_handle_interrupt(controllers[a_controller_index].p_i2c_slave_handle);
    }
    else
    {
        assert(false);
    }
}

void I2C1_EV_IRQHandler()
{
    interupt_handler(0);
}

void I2C2_EV_IRQHandler()
{
    interupt_handler(1);
}

void I2C3_EV_IRQHandler()
{
    interupt_handler(2);
}

void I2C4_EV_IRQHandler()
{
    interupt_handler(3);
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;
using namespace cml::hal::core;
using namespace cml::utils;

void i2c_handle_interrupt(I2C_base* a_p_this)
{
    volatile uint32 isr = a_p_this->p_i2c->ISR;
    volatile uint32 cr1 = a_p_this->p_i2c->CR1;

    if (true == is_flag(isr, I2C_ISR_RXNE) && true == is_flag(cr1, I2C_CR1_RXIE))
    {
        a_p_this->rx_context.index++;

        bool ret = a_p_this->rx_callback.function(static_cast<uint8>(a_p_this->p_i2c->RXDR),
                                                  get_bus_status_from_I2C_ISR(a_p_this->p_i2c, Transfer_type::receive),
                                                  a_p_this->rx_callback.p_user_data);

        if (true == is_I2C_ISR_error(a_p_this->p_i2c))
        {
            clear_I2C_ISR_errors(a_p_this->p_i2c);
        }

        if (false == ret || a_p_this->rx_context.index == a_p_this->rx_context.size)
        {
            a_p_this->rx_callback      = { nullptr, nullptr };
            a_p_this->rx_context.index = 0;
            a_p_this->rx_context.size  = 0;

            set_flag(&(a_p_this->p_i2c->ICR), I2C_ICR_STOPCF);
            clear_flag(&(a_p_this->p_i2c->CR1), I2C_CR1_RXIE);
        }
    }

    if (true == is_flag(isr, I2C_ISR_TXE) && true == is_flag(cr1, I2C_CR1_TXIE))
    {
        a_p_this->tx_context.index++;

        bool ret = a_p_this->tx_callback.function(reinterpret_cast<volatile uint32*>(&(a_p_this->p_i2c->TXDR)),
                                                  get_bus_status_from_I2C_ISR(a_p_this->p_i2c, Transfer_type::transmit),
                                                  a_p_this->tx_callback.p_user_data);

        if (true == is_I2C_ISR_error(a_p_this->p_i2c))
        {
            clear_I2C_ISR_errors(a_p_this->p_i2c);
        }

        if (false == ret || a_p_this->tx_context.index == a_p_this->tx_context.size)
        {
            a_p_this->tx_callback      = { nullptr, nullptr };
            a_p_this->tx_context.index = 0;
            a_p_this->tx_context.size  = 0;

            set_flag(&(a_p_this->p_i2c->ICR), I2C_ICR_STOPCF);
            clear_flag(&(a_p_this->p_i2c->CR1), I2C_CR1_TXIE);
        }
    }

    if (true == is_flag(isr, I2C_ISR_ADDR) && true == is_flag(cr1, I2C_CR1_ADDRIE))
    {
        set_flag(&(a_p_this->p_i2c->ICR), I2C_ICR_ADDRCF);
    }
}

I2C_base::Clock_source I2C_base::get_clock_source() const
{
    return get_clock_source_from_RCC_CCIPR(this->id);
}

void I2C_master::enable(const Config& a_config,
                        Clock_source a_clock_source,
                        uint32 a_irq_priority)
{
    assert(nullptr == controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr == controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle);

    controllers[static_cast<uint32>(this->id)].enable(get_RCC_CCIPR_from_clock_source(a_clock_source, this->id),
                                                                                      a_irq_priority);

    controllers[static_cast<uint32>(this->id)].p_i2c_master_handle = this;
    this->p_i2c = controllers[static_cast<uint32>(this->id)].p_registers;

    this->p_i2c->CR1     = 0;
    this->p_i2c->TIMINGR = a_config.timings;

    this->p_i2c->CR1 = (false == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                       (true == a_config.crc_enable ? I2C_CR1_PECEN : 0)      |
                       I2C_CR1_PE;

    if (true == a_config.fast_plus)
    {
        assert(true == mcu::is_syscfg_enabled());

        set_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32>(this->id));
    }
}

void I2C_master::diasble()
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);

    this->p_i2c->CR1 = 0;

    if (true == this->is_fast_plus())
    {
        clear_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32>(this->id));
    }

    controllers[static_cast<uint32>(this->id)].disable();
    controllers[static_cast<uint32>(this->id)].p_i2c_master_handle = nullptr;

    this->p_i2c = nullptr;
}

uint32 I2C_master::transmit_bytes_polling(uint16 a_slave_address,
                                          const void* a_p_data,
                                          uint32 a_data_size_in_bytes,
                                          Bus_status* a_p_status)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_i2c->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes && false == is_I2C_ISR_error(this->p_i2c))
    {
        if (true == is_flag(this->p_i2c->ISR, I2C_ISR_TXE))
        {
            this->p_i2c->TXDR = static_cast<const byte*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(this->p_i2c, Transfer_type::transmit);
    }

    if (true == is_I2C_ISR_error(this->p_i2c))
    {
        clear_I2C_ISR_errors(this->p_i2c);
    }

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    this->p_i2c->CR2 = 0;

    return ret;
}

uint32 I2C_master::transmit_bytes_polling(uint16 a_slave_address,
                                          const void* a_p_data,
                                          uint32 a_data_size_in_bytes,
                                          time_tick a_timeout_ms,
                                          Bus_status* a_p_status)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(true == systick::is_enabled() && a_timeout_ms > 0);

    time_tick start = systick::get_counter();

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_i2c->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error(this->p_i2c) &&
           a_timeout_ms <= time_tick_diff(hal::systick::get_counter(), start))
    {
        if (true == is_flag(this->p_i2c->ISR, I2C_ISR_TXE))
        {
            this->p_i2c->TXDR = static_cast<const byte*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(this->p_i2c, Transfer_type::transmit);
    }

    if (true == is_I2C_ISR_error(this->p_i2c))
    {
        clear_I2C_ISR_errors(this->p_i2c);
    }

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    this->p_i2c->CR2 = 0;

    return ret;
}

uint32 I2C_master::receive_bytes_polling(uint16 a_slave_address,
                                         void* a_p_data,
                                         uint32 a_data_size_in_bytes,
                                         Bus_status* a_p_status)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_i2c->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes && false == is_I2C_ISR_error(this->p_i2c))
    {
        if (true == is_flag(this->p_i2c->ISR, I2C_ISR_RXNE))
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(this->p_i2c->RXDR);
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(this->p_i2c, Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error(this->p_i2c))
    {
        clear_I2C_ISR_errors(this->p_i2c);
    }

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    this->p_i2c->CR2 = 0;

    return ret;
}

uint32 I2C_master::receive_bytes_polling(uint16 a_slave_address,
                                         void* a_p_data,
                                         uint32 a_data_size_in_bytes,
                                         time_tick a_timeout_ms,
                                         Bus_status* a_p_status)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(true == systick::is_enabled() && a_timeout_ms > 0);

    time_tick start = systick::get_counter();

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_i2c->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error(this->p_i2c) &&
           a_timeout_ms <= time_tick_diff(hal::systick::get_counter(), start))
    {
        if (true == is_flag(this->p_i2c->ISR, I2C_ISR_RXNE))
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(this->p_i2c->RXDR);
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(this->p_i2c, Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error(this->p_i2c))
    {
        clear_I2C_ISR_errors(this->p_i2c);
    }

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    this->p_i2c->CR2 = 0;

    return ret;
}

void I2C_master::start_transmit_bytes_it(uint16 a_slave_address,
                                        const TX_callback& a_callback,
                                        uint32 a_data_size_in_bytes)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    this->tx_context.index = 0;
    this->tx_context.size  = a_data_size_in_bytes;

    this->rx_callback = { nullptr, nullptr };
    this->tx_callback = a_callback;

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_i2c->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;
    set_flag(&(this->p_i2c->CR1), I2C_CR1_TXIE);
}

void I2C_master::start_receive_bytes_it(uint16 a_slave_address,
                                        const RX_callback& a_callback,
                                        uint32 a_data_size_in_bytes)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    this->rx_context.index = 0;
    this->rx_context.size  = a_data_size_in_bytes;

    this->tx_callback = { nullptr, nullptr };
    this->rx_callback = a_callback;

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_i2c->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
    set_flag(&(this->p_i2c->CR1), I2C_CR1_RXIE);
}

void I2C_master::stop_transmit_bytes_it()
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != this->tx_callback.function);

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    clear_flag(&(this->p_i2c->CR1), I2C_CR1_TXIE);

    this->tx_context  = { 0, 0 };
    this->tx_callback = { nullptr, nullptr };
}

void I2C_master::stop_receive_bytes_it()
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != this->rx_callback.function);

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    clear_flag(&(this->p_i2c->CR1), I2C_CR1_RXIE);

    this->rx_context  = { 0, 0 };
    this->rx_callback = { nullptr, nullptr };
}

bool I2C_master::is_slave_connected(uint16 a_slave_address, time_tick a_timeout_ms) const
{
    assert(nullptr != this->p_i2c);
    assert(true == systick::is_enabled());

    time_tick start = systick::get_counter();

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;

    this->p_i2c->CR2 = address_mask | I2C_CR2_AUTOEND | I2C_CR2_START;

    bool ret = wait::until(&(this->p_i2c->ISR), I2C_ISR_STOPF, false, start, a_timeout_ms);

    if (true == ret)
    {
        if (true == is_flag(this->p_i2c->ISR, I2C_ISR_NACKF))
        {
            set_flag(&(this->p_i2c->ICR), I2C_ICR_NACKCF);
            ret = false;
        }
    }

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    this->p_i2c->CR2 = 0;

    return ret;
}

void I2C_slave::enable(const Config& a_config,
                       Clock_source a_clock_source,
                       uint32 a_irq_priority)
{
    assert(nullptr == controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr == controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle);
    assert(a_config.address <= 0x7F);

    controllers[static_cast<uint32>(this->id)].enable(get_RCC_CCIPR_from_clock_source(a_clock_source, this->id),
                                                      a_irq_priority);

    controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle = this;
    this->p_i2c = controllers[static_cast<uint32>(this->id)].p_registers;

    this->p_i2c->CR1     = 0;
    this->p_i2c->TIMINGR = a_config.timings;

    this->p_i2c->OAR1 = I2C_OAR1_OA1EN | (a_config.address << 1);
    this->p_i2c->CR1  = (false == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                        (true == a_config.crc_enable ? I2C_CR1_PECEN : 0)|
                        I2C_CR1_ADDRIE | I2C_CR1_PE;

    if (true == a_config.fast_plus)
    {
        assert(true == mcu::is_syscfg_enabled());

        set_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32>(this->id));
    }
}

void I2C_slave::diasble()
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle);

    this->p_i2c->CR1 = 0;

    if (true == this->is_fast_plus())
    {
        clear_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32>(this->id));
    }

    controllers[static_cast<uint32>(this->id)].disable();
    controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle = nullptr;

    this->p_i2c = nullptr;
}

uint32 I2C_slave::transmit_bytes_polling(const void* a_p_data,
                                         uint32 a_data_size_in_bytes,
                                         Bus_status* a_p_status)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error(this->p_i2c))
    {
        if (true == is_flag(this->p_i2c->ISR, I2C_ISR_TXE))
        {
            this->p_i2c->TXDR = static_cast<const uint8*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(this->p_i2c, Transfer_type::transmit);
    }

    if (true == is_I2C_ISR_error(this->p_i2c))
    {
        clear_I2C_ISR_errors(this->p_i2c);
    }

    return ret;
}

uint32 I2C_slave::transmit_bytes_polling(const void* a_p_data,
                                         uint32 a_data_size_in_bytes,
                                         time_tick a_timeout_ms,
                                         Bus_status* a_p_status)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(true == systick::is_enabled() && a_timeout_ms > 0);

    time_tick start = systick::get_counter();

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error(this->p_i2c) &&
           a_timeout_ms <= time_tick_diff(hal::systick::get_counter(), start))
    {
        if (true == is_flag(this->p_i2c->ISR, I2C_ISR_TXE))
        {
            this->p_i2c->TXDR = static_cast<const uint8*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(this->p_i2c, Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error(this->p_i2c))
    {
        clear_I2C_ISR_errors(this->p_i2c);
    }

    return ret;
}

common::uint32 I2C_slave::receive_bytes_polling(void* a_p_data,
                                                uint32 a_data_size_in_bytes,
                                                Bus_status* a_p_status)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error(this->p_i2c))
    {
        if (true == is_flag(this->p_i2c->ISR, I2C_ISR_RXNE))
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(this->p_i2c->RXDR);
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(this->p_i2c, Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error(this->p_i2c))
    {
        clear_I2C_ISR_errors(this->p_i2c);
    }

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);

    return ret;
}

common::uint32 I2C_slave::receive_bytes_polling(void* a_p_data,
                                                uint32 a_data_size_in_bytes,
                                                time_tick a_timeout_ms,
                                                Bus_status* a_p_status)
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(true == systick::is_enabled() && a_timeout_ms > 0);

    time_tick start = systick::get_counter();


    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error(this->p_i2c) &&
           a_timeout_ms <= time_tick_diff(hal::systick::get_counter(), start))
    {
        if (true == is_flag(this->p_i2c->ISR, I2C_ISR_RXNE))
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(this->p_i2c->RXDR);
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(this->p_i2c, Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error(this->p_i2c))
    {
        clear_I2C_ISR_errors(this->p_i2c);
    }

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);

    return ret;
}

void I2C_slave::stop_transmit_bytes_it()
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != this->tx_callback.function);

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    clear_flag(&(this->p_i2c->CR1), I2C_CR1_TXIE);

    this->tx_context = { 0, 0 };
    this->tx_callback = { nullptr, nullptr };
}

void I2C_slave::stop_receive_bytes_it()
{
    assert(nullptr != this->p_i2c);
    assert(nullptr != this->rx_callback.function);

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    clear_flag(&(this->p_i2c->CR1), I2C_CR1_RXIE);

    this->rx_context = { 0, 0 };
    this->rx_callback = { nullptr, nullptr };
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml