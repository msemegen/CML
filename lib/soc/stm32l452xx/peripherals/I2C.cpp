/*
    Name: I2C.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/peripherals/I2C.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/counter.hpp>
#include <soc/stm32l452xx/mcu.hpp>

// cml
#include <cml/debug/assert.hpp>
#include <cml/utils/wait.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l452xx::peripherals;

struct Controller
{
    using Enable_function  = void (*)(uint32_t a_clock_source, uint32_t a_irq_priority);
    using Disable_function = void (*)();

    I2C_TypeDef* p_registers        = nullptr;
    I2C_master* p_i2c_master_handle = nullptr;
    I2C_slave* p_i2c_slave_handle   = nullptr;

    Enable_function enable   = nullptr;
    Disable_function disable = nullptr;
};

void i2c_1_enable(uint32_t a_clock_source, uint32_t a_irq_priority)
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

void i2c_2_enable(uint32_t a_clock_source, uint32_t a_irq_priority)
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

void i2c_3_enable(uint32_t a_clock_source, uint32_t a_irq_priority)
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

void i2c_4_enable(uint32_t a_clock_source, uint32_t a_irq_priority)
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

Controller controllers[] { { I2C1, nullptr, nullptr, i2c_1_enable, i2c_1_disable },
                           { I2C2, nullptr, nullptr, i2c_2_enable, i2c_2_disable },
                           { I2C3, nullptr, nullptr, i2c_3_enable, i2c_3_disable },
                           { I2C4, nullptr, nullptr, i2c_4_enable, i2c_4_disable } };

bool is_I2C_ISR_error(uint32_t a_isr)
{
    return is_any_bit(a_isr,
                      I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR | I2C_ISR_NACKF);
}

void clear_I2C_ISR_errors(volatile uint32_t* a_p_icr)
{
    set_flag(a_p_icr,
             I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF | I2C_ICR_NACKCF);
}

I2C_base::Bus_status_flag get_bus_status_flag_from_I2C_ISR(uint32_t a_isr)
{
    I2C_base::Bus_status_flag ret = I2C_base::Bus_status_flag::ok;

    if (true == is_flag(a_isr, I2C_ISR_OVR))
    {
        ret |= I2C_base::Bus_status_flag::buffer_error;
    }

    if (true == is_flag(a_isr, I2C_ISR_ARLO))
    {
        ret |= I2C_base::Bus_status_flag::arbitration_lost;
    }

    if (true == is_flag(a_isr, I2C_ISR_BERR))
    {
        ret |= I2C_base::Bus_status_flag::misplaced;
    }

    if (true == is_flag(a_isr, I2C_ISR_NACKF))
    {
        ret |= I2C_base::Bus_status_flag::nack;
    }

    return ret;
}

I2C_base::Clock_source get_clock_source_from_RCC_CCIPR(I2C_base::Id a_id)
{
    switch (a_id)
    {
        case I2C_base::Id::_1:
        case I2C_base::Id::_2:
        case I2C_base::Id::_3: {
            return static_cast<I2C_base::Clock_source>(
                get_flag(RCC->CCIPR, 0x3 << (RCC_CCIPR_I2C1SEL_Pos + static_cast<uint32_t>(a_id) * 2)) >>
                RCC_CCIPR_I2C1SEL_Pos);
        }
        break;

        default: {
            return static_cast<I2C_base::Clock_source>(get_flag(RCC->CCIPR2, RCC_CCIPR2_I2C4SEL));
        }
        break;
    }
}

uint32_t get_RCC_CCIPR_from_clock_source(I2C_base::Clock_source a_clock_source, I2C_base::Id a_i2c_id)
{
    switch (a_i2c_id)
    {
        case I2C_base::Id::_1:
        case I2C_base::Id::_2:
        case I2C_base::Id::_3: {
            return static_cast<uint32_t>(a_clock_source)
                   << (RCC_CCIPR_I2C1SEL_Pos + static_cast<uint32_t>(a_i2c_id) * 2);
        }
        break;

        case I2C_base::Id::_4: {
            return static_cast<uint32_t>(a_clock_source);
        }
        break;
    }

    return 0;
}

I2C_TypeDef* get_i2c_ptr(I2C_base::Id a_id)
{
    return controllers[static_cast<uint32_t>(a_id)].p_registers;
}

} // namespace

extern "C" {

void interupt_handler(uint32_t a_controller_index)
{
    assert(nullptr != controllers[a_controller_index].p_i2c_master_handle ||
           nullptr != controllers[a_controller_index].p_i2c_slave_handle);

    if (nullptr != controllers[a_controller_index].p_i2c_master_handle)
    {
        i2c_master_interrupt_handler(controllers[a_controller_index].p_i2c_master_handle);
    }
    else if (nullptr != controllers[a_controller_index].p_i2c_slave_handle)
    {
        i2c_slave_interrupt_handler(controllers[a_controller_index].p_i2c_slave_handle);
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

namespace soc {
namespace stm32l452xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void i2c_master_interrupt_handler(I2C_master* a_p_this)
{
    const uint32_t isr = get_i2c_ptr(a_p_this->id)->ISR;
    const uint32_t cr1 = get_i2c_ptr(a_p_this->id)->CR1;

    if (true == is_I2C_ISR_error(isr))
    {
        const I2C_base::Bus_status_flag status = get_bus_status_flag_from_I2C_ISR(isr);

        if (I2C_base::Bus_status_flag::ok != status)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            }

            clear_I2C_ISR_errors(&(get_i2c_ptr(a_p_this->id)->ICR));
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == is_flag(isr, I2C_ISR_RXNE) && true == is_flag(cr1, I2C_CR1_RXIE))
        {
            a_p_this->receive_callback.function(static_cast<uint8_t>(get_i2c_ptr(a_p_this->id)->RXDR),
                                                false,
                                                a_p_this,
                                                a_p_this->receive_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == is_flag(isr, I2C_ISR_TXE) && true == is_flag(cr1, I2C_CR1_TXIE))
        {
            a_p_this->transmit_callback.function(
                reinterpret_cast<volatile uint32_t*>(&(get_i2c_ptr(a_p_this->id)->TXDR)),
                false,
                a_p_this,
                a_p_this->transmit_callback.p_user_data);
        }
    }

    if (true == is_flag(isr, I2C_ISR_STOPF) && true == is_flag(cr1, I2C_CR1_STOPIE))
    {
        set_flag(&(get_i2c_ptr(a_p_this->id)->ICR), I2C_ICR_STOPCF);

        if (nullptr != a_p_this->receive_callback.function)
        {
            a_p_this->receive_callback.function(0, true, a_p_this, a_p_this->receive_callback.p_user_data);
        }

        if (nullptr != a_p_this->transmit_callback.function)
        {
            a_p_this->transmit_callback.function(nullptr, true, a_p_this, a_p_this->transmit_callback.p_user_data);
        }
    }
}

void i2c_slave_interrupt_handler(I2C_slave* a_p_this)
{
    const uint32_t isr = get_i2c_ptr(a_p_this->id)->ISR;
    const uint32_t cr1 = get_i2c_ptr(a_p_this->id)->CR1;

    if (true == is_flag(isr, I2C_ISR_NACKF) && nullptr != a_p_this->transmit_callback.function)
    {
        set_flag(&(get_i2c_ptr(a_p_this->id)->ICR), I2C_ICR_NACKCF);
    }
    else
    {
        const I2C_base::Bus_status_flag status = get_bus_status_flag_from_I2C_ISR(isr);

        if (I2C_base::Bus_status_flag::ok != status)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            }
            clear_I2C_ISR_errors(&(get_i2c_ptr(a_p_this->id)->ICR));
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == is_flag(isr, I2C_ISR_RXNE) && true == is_flag(cr1, I2C_CR1_RXIE))
        {
            a_p_this->receive_callback.function(static_cast<uint8_t>(get_i2c_ptr(a_p_this->id)->RXDR),
                                                false,
                                                a_p_this,
                                                a_p_this->receive_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == is_flag(isr, I2C_ISR_TXE) && true == is_flag(cr1, I2C_CR1_TXIE))
        {
            a_p_this->transmit_callback.function(
                reinterpret_cast<volatile uint32_t*>(&(get_i2c_ptr(a_p_this->id)->TXDR)),
                false,
                a_p_this,
                a_p_this->transmit_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->address_match_callback.function)
    {
        if (true == is_flag(isr, I2C_ISR_ADDR) && true == is_flag(cr1, I2C_CR1_ADDRIE))
        {
            a_p_this->address_match_callback.function(a_p_this, a_p_this->address_match_callback.p_user_data);

            set_flag(&(get_i2c_ptr(a_p_this->id)->ICR), I2C_ICR_ADDRCF);
        }
    }

    if (true == is_flag(isr, I2C_ISR_STOPF) && true == is_flag(cr1, I2C_CR1_STOPIE))
    {
        set_flag(&(get_i2c_ptr(a_p_this->id)->ICR), I2C_ICR_STOPCF);

        if (nullptr != a_p_this->receive_callback.function)
        {
            a_p_this->receive_callback.function(0, true, a_p_this, a_p_this->receive_callback.p_user_data);
        }

        if (nullptr != a_p_this->transmit_callback.function)
        {
            a_p_this->transmit_callback.function(nullptr, true, a_p_this, a_p_this->transmit_callback.p_user_data);
        }
    }
}

bool I2C_base::is_analog_filter() const
{
    return false == cml::is_flag(get_i2c_ptr(this->id)->CR1, I2C_CR1_ANFOFF);
}

uint32_t I2C_base::get_timing() const
{
    return get_i2c_ptr(this->id)->TIMINGR;
}

bool I2C_base::is_fast_plus() const
{
    return cml::is_bit(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id));
}

bool I2C_base::is_enabled() const
{
    return is_flag(controllers[static_cast<uint32_t>(this->id)].p_registers->CR1, I2C_CR1_PE);
}

I2C_base::Clock_source I2C_base::get_clock_source() const
{
    return get_clock_source_from_RCC_CCIPR(this->id);
}

void I2C_master::enable(const Config& a_config, Clock_source a_clock_source, uint32_t a_irq_priority)
{
    controllers[static_cast<uint32_t>(this->id)].enable(get_RCC_CCIPR_from_clock_source(a_clock_source, this->id),
                                                        a_irq_priority);

    controllers[static_cast<uint32_t>(this->id)].p_i2c_master_handle = this;

    get_i2c_ptr(this->id)->CR1     = 0;
    get_i2c_ptr(this->id)->TIMINGR = a_config.timings;

    get_i2c_ptr(this->id)->CR1 = (false == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                                 (true == a_config.crc_enable ? I2C_CR1_PECEN : 0) | I2C_CR1_PE;

    if (true == a_config.fast_plus)
    {
        assert(true == mcu::is_syscfg_enabled());

        set_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id));
    }
}

void I2C_master::diasble()
{
    get_i2c_ptr(this->id)->CR1 = 0;

    if (true == this->is_fast_plus())
    {
        clear_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id));
    }

    controllers[static_cast<uint32_t>(this->id)].disable();
    controllers[static_cast<uint32_t>(this->id)].p_i2c_master_handle = nullptr;
}

I2C_master::Result
I2C_master::transmit_bytes_polling(uint16_t a_slave_address, const void* a_p_data, uint32_t a_data_size_in_bytes)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    const uint32_t address_mask = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask =
        (static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_TXE) && words < a_data_size_in_bytes)
        {
            get_i2c_ptr(this->id)->TXDR = static_cast<const uint8_t*>(a_p_data)[words++];
        }

        error = is_I2C_ISR_error(get_i2c_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(get_i2c_ptr(this->id)->ISR);
        clear_I2C_ISR_errors(&(get_i2c_ptr(this->id)->ICR));
    }

    set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    get_i2c_ptr(this->id)->CR2 = 0;

    return { bus_status, words };
}

I2C_master::Result I2C_master::transmit_bytes_polling(uint16_t a_slave_address,
                                                      const void* a_p_data,
                                                      uint32_t a_data_size_in_bytes,
                                                      time::tick a_timeout)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= time::diff(counter::get(), start))
    {
        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_TXE) && words < a_data_size_in_bytes)
        {
            get_i2c_ptr(this->id)->TXDR = static_cast<const uint8_t*>(a_p_data)[words++];
        }

        error = is_I2C_ISR_error(get_i2c_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(get_i2c_ptr(this->id)->ISR);
        clear_I2C_ISR_errors(&(get_i2c_ptr(this->id)->ICR));
    }

    set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    get_i2c_ptr(this->id)->CR2 = 0;

    return { bus_status, words };
}

I2C_master::Result
I2C_master::receive_bytes_polling(uint16_t a_slave_address, void* a_p_data, uint32_t a_data_size_in_bytes)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE) && words < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);
        }

        error = is_I2C_ISR_error(get_i2c_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(get_i2c_ptr(this->id)->ISR);
        clear_I2C_ISR_errors(&(get_i2c_ptr(this->id)->ICR));
    }

    set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    get_i2c_ptr(this->id)->CR2 = 0;

    return { bus_status, words };
}

I2C_master::Result I2C_master::receive_bytes_polling(uint16_t a_slave_address,
                                                     void* a_p_data,
                                                     uint32_t a_data_size_in_bytes,
                                                     time::tick a_timeout)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= time::diff(counter::get(), start))
    {
        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE) && words < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);
        }

        error = is_I2C_ISR_error(get_i2c_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(get_i2c_ptr(this->id)->ISR);
        clear_I2C_ISR_errors(&(get_i2c_ptr(this->id)->ICR));
    }

    set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    get_i2c_ptr(this->id)->CR2 = 0;

    return { bus_status, words };
}

void I2C_master::register_transmit_callback(uint16_t a_slave_address,
                                            const Transmit_callback& a_callback,
                                            uint32_t a_data_length_in_bytes)
{
    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_length_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;
    set_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);
}

void I2C_master::register_receive_callback(uint16_t a_slave_address,
                                           const Receive_callback& a_callback,
                                           uint32_t a_data_length_in_bytes)
{
    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_length_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
    set_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);
}

void I2C_master::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;
    set_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);
}

void I2C_master::unregister_transmit_callback()
{
    assert(nullptr != this->transmit_callback.function);

    Interrupt_guard guard;

    clear_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);

    this->transmit_callback = { nullptr, nullptr };
}

void I2C_master::unregister_receive_callback()
{
    assert(nullptr != this->receive_callback.function);

    Interrupt_guard guard;

    clear_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);

    this->receive_callback = { nullptr, nullptr };
}

void I2C_master::unregister_bus_status_callback()
{
    assert(nullptr != this->bus_status_callback.function);

    Interrupt_guard guard;

    clear_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);

    this->bus_status_callback = { nullptr, nullptr };
}

bool I2C_master::is_slave_connected(uint16_t a_slave_address, time::tick a_timeout) const
{
    assert(a_timeout > 0);

    time::tick start = counter::get();

    const uint32_t address_mask = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;

    get_i2c_ptr(this->id)->CR2 = address_mask | I2C_CR2_AUTOEND | I2C_CR2_START;

    bool ret = wait::until(&(get_i2c_ptr(this->id)->ISR), I2C_ISR_STOPF, false, start, a_timeout);

    if (true == ret)
    {
        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_NACKF))
        {
            set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_NACKCF);
            ret = false;
        }
    }

    set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    get_i2c_ptr(this->id)->CR2 = 0;

    return ret;
}

void I2C_slave::enable(const Config& a_config, Clock_source a_clock_source, uint32_t a_irq_priority)
{
    assert(a_config.address <= 0x7F);

    controllers[static_cast<uint32_t>(this->id)].enable(get_RCC_CCIPR_from_clock_source(a_clock_source, this->id),
                                                        a_irq_priority);
    controllers[static_cast<uint32_t>(this->id)].p_i2c_slave_handle = this;

    get_i2c_ptr(this->id)->CR1     = 0;
    get_i2c_ptr(this->id)->TIMINGR = a_config.timings;
    get_i2c_ptr(this->id)->OAR1    = I2C_OAR1_OA1EN | (a_config.address << 1);
    get_i2c_ptr(this->id)->CR1     = (false == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                                 (true == a_config.crc_enable ? I2C_CR1_PECEN : 0) | I2C_CR1_PE;

    if (true == a_config.fast_plus)
    {
        assert(true == mcu::is_syscfg_enabled());

        set_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id));
    }
}

void I2C_slave::diasble()
{
    get_i2c_ptr(this->id)->CR1 = 0;

    if (true == this->is_fast_plus())
    {
        clear_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id));
    }

    controllers[static_cast<uint32_t>(this->id)].disable();
    controllers[static_cast<uint32_t>(this->id)].p_i2c_slave_handle = nullptr;
}

I2C_slave::Result I2C_slave::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_bytes)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    constexpr uint32_t error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while ((false == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) &&
            false == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF)) &&
           false == error)
    {
        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_ADDR))
        {
            set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_ADDRCF);
        }

        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_TXE) && words < a_data_size_in_bytes)
        {
            get_i2c_ptr(this->id)->TXDR = static_cast<const uint8_t*>(a_p_data)[words++];
        }

        error = is_any_bit(I2C1->ISR, error_mask);
    }

    if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) &&
        true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_NACKF))
    {
        set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(get_i2c_ptr(this->id)->ISR);
        clear_I2C_ISR_errors(&(get_i2c_ptr(this->id)->ICR));
    }

    set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);

    return { bus_status, words };
}

I2C_slave::Result
I2C_slave::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_bytes, time::tick a_timeout)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    constexpr uint32_t error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while ((false == is_flag(I2C1->ISR, I2C_ISR_STOPF) && false == is_flag(I2C1->ISR, I2C_ISR_STOPF)) &&
           false == error && a_timeout >= time::diff(counter::get(), start))
    {
        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_ADDR))
        {
            set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_ADDRCF);
        }

        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_TXE) && words < a_data_size_in_bytes)
        {
            get_i2c_ptr(this->id)->TXDR = static_cast<const uint8_t*>(a_p_data)[words++];
        }

        error = is_any_bit(I2C1->ISR, error_mask);
    }

    if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) &&
        true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_NACKF))
    {
        set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(get_i2c_ptr(this->id)->ISR);
        clear_I2C_ISR_errors(&(get_i2c_ptr(this->id)->ICR));
    }

    set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);

    return { bus_status, words };
}

I2C_slave::Result I2C_slave::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_ADDR))
        {
            set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_ADDRCF);
        }

        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE))
        {
            const uint8_t rxdr = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);

            if (words < a_data_size_in_bytes)
            {
                static_cast<uint8_t*>(a_p_data)[words++] = rxdr;
            }
        }

        error = is_I2C_ISR_error(get_i2c_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(get_i2c_ptr(this->id)->ISR);
        clear_I2C_ISR_errors(&(get_i2c_ptr(this->id)->ICR));
    }

    set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);

    return { bus_status, words };
}

I2C_slave::Result I2C_slave::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes, time::tick a_timeout)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(I2C1->ICR, I2C_ICR_STOPCF) && false == error &&
           a_timeout >= time::diff(counter::get(), start))
    {
        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_ADDR))
        {
            set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_ADDRCF);
        }

        if (true == is_flag(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE))
        {
            const uint8_t rxdr = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);

            if (words < a_data_size_in_bytes)
            {
                static_cast<uint8_t*>(a_p_data)[words++] = rxdr;
            }
        }

        error = is_I2C_ISR_error(get_i2c_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(get_i2c_ptr(this->id)->ISR);
        clear_I2C_ISR_errors(&(get_i2c_ptr(this->id)->ICR));
    }

    set_flag(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);

    return { bus_status, words };
}

void I2C_slave::register_transmit_callback(const Transmit_callback& a_callback, uint32_t a_data_size_in_bytes)
{
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    set_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE | I2C_CR1_NACKIE);
}

void I2C_slave::register_receive_callback(const Receive_callback& a_callback, uint32_t a_data_size_in_bytes)
{
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    set_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);
}

void I2C_slave::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;
    set_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_NACKIE | I2C_CR1_ADDRIE | I2C_CR1_ERRIE);
}

void I2C_slave::register_address_match_callback(const Addres_match_callback& a_callback)
{
    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->address_match_callback = a_callback;

    set_flag(&(I2C1->CR1), I2C_CR1_ADDRIE);
}

void I2C_slave::unregister_transmit_callback()
{
    Interrupt_guard guard;

    clear_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);

    this->transmit_callback = { nullptr, nullptr };
}

void I2C_slave::unregister_receive_callback()
{
    assert(nullptr != this->receive_callback.function);

    Interrupt_guard guard;

    clear_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);

    this->receive_callback = { nullptr, nullptr };
}

void I2C_slave::unregister_bus_status_callback()
{
    assert(nullptr != this->bus_status_callback.function);

    Interrupt_guard guard;

    clear_flag(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_NACKIE | I2C_CR1_ADDRIE | I2C_CR1_ERRIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void I2C_slave::unregister_address_match_callback()
{
    Interrupt_guard guard;

    clear_flag(&(I2C1->CR1), I2C_CR1_ADDRIE);

    this->address_match_callback = { nullptr, nullptr };
}

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx