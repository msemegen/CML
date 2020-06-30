/*
    Name: I2C.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

//this
#include <soc/stm32l011xx/peripherals/I2C.hpp>

//soc
#include <soc/counter.hpp>
#include <soc/stm32l011xx/mcu.hpp>

//cml
#include <cml/utils/wait.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l011xx::peripherals;

struct Controller
{
    I2C_master* p_i2c_master_handle = nullptr;
    I2C_slave* p_i2c_slave_handle   = nullptr;
};

void i2c_1_enable(uint32 a_clock_source, uint32 a_irq_priority)
{
    assert(0 != a_clock_source);

    set_flag(&(RCC->CCIPR), RCC_CCIPR_I2C1SEL, a_clock_source);
    set_flag(&(RCC->APB1ENR), (RCC_APB1ENR_I2C1EN));

    NVIC_SetPriority(I2C1_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C1_IRQn);
}

void i2c_1_disable()
{
    clear_flag(&(RCC->APB1ENR), (RCC_APB1ENR_I2C1EN));
    NVIC_DisableIRQ(I2C1_IRQn);
}

bool is_I2C_ISR_error(uint32 a_isr)
{
    return is_any_bit(a_isr, I2C_ISR_TIMEOUT |
                             I2C_ISR_PECERR  |
                             I2C_ISR_OVR     |
                             I2C_ISR_ARLO    |
                             I2C_ISR_BERR    |
                             I2C_ISR_NACKF);
}

void clear_I2C_ISR_errors(volatile uint32* a_p_icr)
{
    set_flag(a_p_icr, I2C_ICR_TIMOUTCF |
                      I2C_ICR_PECCF    |
                      I2C_ICR_OVRCF    |
                      I2C_ICR_ARLOCF   |
                      I2C_ICR_BERRCF   |
                      I2C_ICR_NACKCF);
}

I2C_base::Bus_status_flag get_bus_status_flag_from_I2C_ISR(uint32 a_isr)
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

Controller controller;

} // namespace ::

extern "C"
{

void I2C1_IRQHandler()
{
    if (nullptr != controller.p_i2c_master_handle)
    {
        i2c_master_interrupt_handler(controller.p_i2c_master_handle);
    }
    else if (nullptr != controller.p_i2c_slave_handle)
    {
        i2c_slave_interrupt_handler(controller.p_i2c_slave_handle);
    }
    else
    {
        assert(false);
    }
}

} // extern "C"

namespace soc {
namespace stm32l011xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;
using namespace cml::collection;

void I2C_base::bus_status_interrupt_handler(uint32 a_isr)
{
    if (nullptr != this->bus_status_callback.function)
    {
        I2C_base::Bus_status_flag status = get_bus_status_flag_from_I2C_ISR(a_isr);

        if (I2C_base::Bus_status_flag::ok != status &&
            true == this->bus_status_callback.function(status, this->bus_status_callback.p_user_data))
        {
            clear_I2C_ISR_errors(&(USART2->ICR));
        }
    }
}

void I2C_base::rxne_interrupt_handler(uint32 a_isr, uint32 a_cr1)
{
    if (true == is_flag(a_isr, I2C_ISR_RXNE) && true == is_flag(a_cr1, I2C_CR1_RXIE))
    {
        this->rx_callback.function(static_cast<uint8>(I2C1->RXDR), false, this->rx_callback.p_user_data);
    }
}

void I2C_base::txe_interrupt_handler(uint32 a_isr, uint32 a_cr1)
{
    if (true == is_flag(a_isr, I2C_ISR_TXE) && true == is_flag(a_cr1, I2C_CR1_TXIE))
    {
        this->tx_callback.function(reinterpret_cast<volatile uint32*>(&(I2C1->TXDR)),
                                   false,
                                   this->tx_callback.p_user_data);
    }
}

void I2C_base::stopf_interrupt_handler(uint32 a_isr, uint32 a_cr1)
{
    if (true == is_flag(a_isr, I2C_ISR_STOPF) && true == is_flag(a_cr1, I2C_CR1_STOPIE))
    {
        if (nullptr != this->tx_callback.function)
        {
            this->tx_callback.function(nullptr, true, this->tx_callback.p_user_data);

            clear_flag(&(I2C1->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);

            this->tx_callback = { nullptr, nullptr };
        }

        if (nullptr != this->rx_callback.function)
        {
            this->rx_callback.function(0, true, this->rx_callback.p_user_data);

            clear_flag(&(I2C1->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);

            this->rx_callback = { nullptr, nullptr };
        }

        set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    }
}

void i2c_master_interrupt_handler(I2C_master* a_p_this)
{
    volatile uint32 isr = I2C1->ISR;
    volatile uint32 cr1 = I2C1->CR1;

    a_p_this->bus_status_interrupt_handler(isr);
    a_p_this->rxne_interrupt_handler(isr, cr1);
    a_p_this->txe_interrupt_handler(isr, cr1);
    a_p_this->stopf_interrupt_handler(isr, cr1);
}

void i2c_slave_interrupt_handler(I2C_slave* a_p_this)
{
    volatile uint32 isr = I2C1->ISR;
    volatile uint32 cr1 = I2C1->CR1;

    if (true == is_flag(isr, I2C_ISR_NACKF) &&
        nullptr != a_p_this->tx_callback.function)
    {
        set_flag(&(I2C1->ICR), I2C_ICR_NACKCF);
    }
    else
    {
        a_p_this->bus_status_interrupt_handler(isr);
    }

    a_p_this->rxne_interrupt_handler(isr, cr1);
    a_p_this->txe_interrupt_handler(isr, cr1);
    a_p_this->stopf_interrupt_handler(isr, cr1);

    if (true == is_flag(isr, I2C_ISR_ADDR) && true == is_flag(cr1, I2C_CR1_ADDRIE))
    {
        set_flag(&(I2C1->ICR), I2C_ICR_ADDRCF);
    }
}

I2C_base::Clock_source I2C_base::get_clock_source() const
{
    return static_cast<Clock_source>(get_flag(RCC->CCIPR, RCC_CCIPR_I2C1SEL) >> RCC_CCIPR_I2C1SEL_Pos);
}

void I2C_master::enable(const Config& a_config, Clock_source a_clock_source, uint32 a_irq_priority)
{
    assert(false == this->is_enabled());

    assert(nullptr == controller.p_i2c_master_handle);
    assert(nullptr == controller.p_i2c_slave_handle);

    i2c_1_enable(static_cast<uint32>(a_clock_source) << RCC_CCIPR_I2C1SEL_Pos, a_irq_priority);
    controller.p_i2c_master_handle = this;

    I2C1->CR1     = 0;
    I2C1->TIMINGR = a_config.timings;

    I2C1->CR1 = (false == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                (true == a_config.crc_enable ? I2C_CR1_PECEN : 0)      |
                I2C_CR1_PE;

    if (true == a_config.fast_plus)
    {
        assert(true == mcu::is_syscfg_enabled());

        set_bit(&(SYSCFG->CFGR2), SYSCFG_CFGR2_I2C1_FMP_Pos);
    }
}

void I2C_master::diasble()
{
    assert(nullptr != controller.p_i2c_master_handle);

    I2C1->CR1 = 0;

    if (true == this->is_fast_plus())
    {
        clear_bit(&(SYSCFG->CFGR2), SYSCFG_CFGR2_I2C1_FMP_Pos);
    }

    i2c_1_disable();
    controller.p_i2c_master_handle = nullptr;
}

I2C_master::Result I2C_master::transmit_bytes_polling(uint16 a_slave_address,
                                                      const void* a_p_data,
                                                      uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = (static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk ;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32 ret = 0;
    bool error = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(I2C1->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_TXE) && ret < a_data_size_in_bytes)
        {
            I2C1->TXDR = static_cast<const byte*>(a_p_data)[ret++];
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return { bus_status, ret };
}

I2C_master::Result I2C_master::transmit_bytes_polling(uint16 a_slave_address,
                                                      const void* a_p_data,
                                                      uint32 a_data_size_in_bytes,
                                                      time::tick a_timeout)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32 ret = 0;
    bool error = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(I2C1->ISR, I2C_ISR_STOPF) &&
           false == error &&
           a_timeout >= time::diff(counter::get(), start))
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_TXE) && ret < a_data_size_in_bytes)
        {
            I2C1->TXDR = static_cast<const byte*>(a_p_data)[ret++];
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return { bus_status, ret };
}

I2C_master::Result I2C_master::receive_bytes_polling(uint16 a_slave_address,
                                                     void* a_p_data,
                                                     uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32 ret = 0;
    bool error = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(I2C1->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_RXNE) && ret < a_data_size_in_bytes)
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(I2C1->RXDR);
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return { bus_status, ret };
}

I2C_master::Result I2C_master::receive_bytes_polling(uint16 a_slave_address,
                                                     void* a_p_data,
                                                     uint32 a_data_size_in_bytes,
                                                     time::tick a_timeout)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32 ret = 0;
    bool error = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(I2C1->ISR, I2C_ISR_STOPF) &&
           false == error &&
           a_timeout >= time::diff(counter::get(), start))
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_RXNE) && ret < a_data_size_in_bytes)
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(I2C1->RXDR);
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return { bus_status, ret };
}

void I2C_master::register_transmit_callback(uint16 a_slave_address,
                                            const TX_callback& a_callback,
                                            uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    this->rx_callback = { nullptr, nullptr };
    this->tx_callback = a_callback;

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;
    set_flag(&(I2C1->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);
}

void I2C_master::register_receive_callback(uint16 a_slave_address,
                                           const RX_callback& a_callback,
                                           uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    this->tx_callback = { nullptr, nullptr };
    this->rx_callback = a_callback;

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
    set_flag(&(I2C1->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);
}

void I2C_master::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    assert(nullptr != a_callback.function);

    this->bus_status_callback = a_callback;
    set_flag(&(I2C1->CR1), I2C_CR1_NACKIE);
}

void I2C_master::unregister_bus_status_callback()
{
    clear_flag(&(I2C1->CR1), I2C_CR1_NACKIE);

    this->bus_status_callback = { nullptr, nullptr };
}

bool I2C_master::is_slave_connected(uint16 a_slave_address, time::tick a_timeout) const
{
    assert(a_timeout > 0);

    time::tick start = counter::get();

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;

    I2C1->CR2 = address_mask | I2C_CR2_AUTOEND | I2C_CR2_START;

    bool ret = wait::until(&(I2C1->ISR), I2C_ISR_STOPF, false, start, a_timeout);

    if (true == ret)
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_NACKF))
        {
            set_flag(&(I2C1->ICR), I2C_ICR_NACKCF);
            ret = false;
        }
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return ret;
}

void I2C_slave::enable(const Config& a_config, Clock_source a_clock_source, uint32 a_irq_priority)
{
    assert(false == this->is_enabled());

    assert(nullptr == controller.p_i2c_master_handle);
    assert(nullptr == controller.p_i2c_slave_handle);
    assert(a_config.address <= 0x7F);

    i2c_1_enable(static_cast<uint32>(a_clock_source) << RCC_CCIPR_I2C1SEL_Pos, a_irq_priority);
    controller.p_i2c_slave_handle = this;

    I2C1->CR1     = 0;
    I2C1->TIMINGR = a_config.timings;

    I2C1->OAR1 = I2C_OAR1_OA1EN | (a_config.address << 1);
    I2C1->CR1  = (false == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                 (true == a_config.crc_enable ? I2C_CR1_PECEN : 0)|
                 I2C_CR1_PE;

    if (true == a_config.fast_plus)
    {
        assert(true == mcu::is_syscfg_enabled());

        set_bit(&(SYSCFG->CFGR2), SYSCFG_CFGR2_I2C1_FMP_Pos);
    }
}

void I2C_slave::diasble()
{
    assert(nullptr != controller.p_i2c_slave_handle);

    I2C1->CR1 = 0;

    if (true == this->is_fast_plus())
    {
        clear_bit(&(SYSCFG->CFGR2), SYSCFG_CFGR2_I2C1_FMP_Pos);
    }

    i2c_1_disable();
    controller.p_i2c_slave_handle = nullptr;
}

I2C_slave::Result I2C_slave::transmit_bytes_polling(const void* a_p_data, uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    constexpr uint32 error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    uint32 ret = 0;
    bool error = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while ((false == is_flag(I2C1->ISR, I2C_ISR_STOPF) && false == is_flag(I2C1->ISR, I2C_ISR_STOPF)) &&
           false == error)
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_ADDR))
        {
            set_flag(&(I2C1->ICR), I2C_ICR_ADDRCF);
        }

        if (true == is_flag(I2C1->ISR, I2C_ISR_TXE) && ret < a_data_size_in_bytes)
        {
            I2C1->TXDR = static_cast<const uint8*>(a_p_data)[ret++];
        }

        error = is_any_bit(I2C1->ISR, error_mask);
    }

    if (true == is_flag(I2C1->ISR, I2C_ISR_STOPF) &&
        true == is_flag(I2C1->ISR, I2C_ISR_NACKF))
    {
        set_flag(&(I2C1->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);

    return { bus_status, ret };
}

I2C_slave::Result I2C_slave::transmit_bytes_polling(const void* a_p_data,
                                                    uint32 a_data_size_in_bytes,
                                                    time::tick a_timeout)
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    constexpr uint32 error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    uint32 ret = 0;
    bool error = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while ((false == is_flag(I2C1->ISR, I2C_ISR_STOPF) && false == is_flag(I2C1->ISR, I2C_ISR_STOPF)) &&
           false == error &&
           a_timeout >= time::diff(counter::get(), start))
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_ADDR))
        {
            set_flag(&(I2C1->ICR), I2C_ICR_ADDRCF);
        }

        if (true == is_flag(I2C1->ISR, I2C_ISR_TXE) && ret < a_data_size_in_bytes)
        {
            I2C1->TXDR = static_cast<const uint8*>(a_p_data)[ret++];
        }

        error = is_any_bit(I2C1->ISR, error_mask);
    }

    if (true == is_flag(I2C1->ISR, I2C_ISR_STOPF) &&
        true == is_flag(I2C1->ISR, I2C_ISR_NACKF))
    {
        set_flag(&(I2C1->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);

    return { bus_status, ret };
}

I2C_slave::Result I2C_slave::receive_bytes_polling(void* a_p_data, uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 ret = 0;
    bool error = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(I2C1->ISR, I2C_ISR_STOPF) &&
           false == error)
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_ADDR))
        {
            set_flag(&(I2C1->ICR), I2C_ICR_ADDRCF);
        }

        if (true == is_flag(I2C1->ISR, I2C_ISR_RXNE))
        {
            if (ret < a_data_size_in_bytes)
            {
                static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(I2C1->RXDR);
            }
            else
            {
                volatile uint32 temp = I2C1->RXDR;
                temp = temp;
                ret++;
            }
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);

    return { bus_status, ret };
}

I2C_slave::Result I2C_slave::receive_bytes_polling(void* a_p_data, uint32 a_data_size_in_bytes, time::tick a_timeout)
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    uint32 ret = 0;
    bool error = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(I2C1->ICR, I2C_ICR_STOPCF) &&
           false == error &&
           a_timeout >= time::diff(counter::get(), start))
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_ADDR))
        {
            set_flag(&(I2C1->ICR), I2C_ICR_ADDRCF);
        }

        if (true == is_flag(I2C1->ISR, I2C_ISR_RXNE))
        {
            if (ret < a_data_size_in_bytes)
            {
                static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(I2C1->RXDR);
            }
            else
            {
                volatile uint32 temp = I2C1->RXDR;
                temp = temp;
                ret++;
            }
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);

    return { bus_status, ret };
}

void I2C_slave::register_transmit_callback(const TX_callback& a_callback, uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    this->rx_callback = { nullptr, nullptr };
    this->tx_callback = a_callback;

    set_flag(&(I2C1->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE | I2C_CR1_NACKIE);
}

void I2C_slave::register_receive_callback(const RX_callback& a_callback, uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    this->tx_callback = { nullptr, nullptr };
    this->rx_callback = a_callback;

    set_flag(&(I2C1->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);
}

void I2C_slave::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    assert(nullptr != a_callback.function);

    this->bus_status_callback = a_callback;
    set_flag(&(I2C1->CR1), I2C_CR1_NACKIE | I2C_CR1_ADDRIE);
}

void I2C_slave::unregister_bus_status_callback()
{
    clear_flag(&(I2C1->CR1), I2C_CR1_NACKIE | I2C_CR1_ADDRIE);

    this->bus_status_callback = { nullptr, nullptr };
}

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx