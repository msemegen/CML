/*
    Name: I2C.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <hal/stm32l011xx/I2C.hpp>

//cml
#include <hal/core/systick.hpp>
#include <hal/stm32l011xx/mcu.hpp>
#include <utils/wait.hpp>

namespace {

using namespace cml::common;
using namespace cml::hal::stm32l011xx;

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

enum class Transfer_type
{
    transmit,
    receive
};

struct Controller
{
    I2C_master* p_i2c_master_handle = nullptr;
    I2C_slave* p_i2c_slave_handle   = nullptr;
};

bool is_I2C_ISR_error()
{
    return is_any_bit(I2C1->ISR, I2C_ISR_TIMEOUT |
                                 I2C_ISR_PECERR  |
                                 I2C_ISR_OVR     |
                                 I2C_ISR_ARLO    |
                                 I2C_ISR_BERR    |
                                 I2C_ISR_NACKF);
}

void clear_I2C_ISR_errors()
{
    set_flag(&(I2C1->ICR), I2C_ICR_TIMOUTCF |
                           I2C_ICR_PECCF    |
                           I2C_ICR_OVRCF    |
                           I2C_ICR_ARLOCF   |
                           I2C_ICR_BERRCF   |
                           I2C_ICR_NACKCF);
}


I2C_base::Bus_status get_bus_status_from_I2C_ISR(Transfer_type a_transfer_type)
{
    if (true == is_flag(I2C1->ISR, I2C_ISR_TIMEOUT))
    {
        return I2C_base::Bus_status::timeout;
    }

    if (true == is_flag(I2C1->ISR, I2C_ISR_OVR))
    {
        switch (a_transfer_type)
        {
            case Transfer_type::receive:
                return I2C_base::Bus_status::overrun;

            case Transfer_type::transmit:
                return I2C_base::Bus_status::underrun;
        }
    }

    if (true == is_flag(I2C1->ISR, I2C_ISR_ARLO))
    {
        return I2C_base::Bus_status::arbitration_lost;
    }

    if (true == is_flag(I2C1->ISR, I2C_ISR_BERR))
    {
        return I2C_base::Bus_status::misplaced;
    }

    if (true == is_flag(I2C1->ISR, I2C_ISR_NACKF))
    {
        return I2C_base::Bus_status::nack;
    }

    return I2C_base::Bus_status::ok;
}

Controller controller;

} // namespace ::

extern "C"
{

void I2C1_IRQHandler()
{
    if (nullptr != controller.p_i2c_master_handle)
    {
        i2c_handle_interrupt(controller.p_i2c_master_handle);
    }
    else if (nullptr != controller.p_i2c_slave_handle)
    {
        i2c_handle_interrupt(controller.p_i2c_slave_handle);
    }
    else
    {
        assert(false);
    }
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l011xx {

using namespace cml::common;
using namespace cml::hal::core;
using namespace cml::utils;

void i2c_handle_interrupt(I2C_base* a_p_this)
{
    volatile uint32 isr = I2C1->ISR;
    volatile uint32 cr1 = I2C1->CR1;

    if (true == is_flag(isr, I2C_ISR_RXNE) && true == is_flag(cr1, I2C_CR1_RXIE))
    {
        a_p_this->rx_context.index++;

        bool ret = a_p_this->rx_callback.function(static_cast<uint8>(I2C1->RXDR),
                                                  get_bus_status_from_I2C_ISR(Transfer_type::receive),
                                                  a_p_this->rx_callback.p_user_data);

        if (true == is_I2C_ISR_error())
        {
            clear_I2C_ISR_errors();
        }

        if (false == ret || a_p_this->rx_context.index == a_p_this->rx_context.size)
        {
            a_p_this->rx_callback      = { nullptr, nullptr };
            a_p_this->rx_context.index = 0;
            a_p_this->rx_context.size  = 0;

            set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
            clear_flag(&(I2C1->CR1), I2C_CR1_RXIE);
        }
    }

    if (true == is_flag(isr, I2C_ISR_TXE) && true == is_flag(cr1, I2C_CR1_TXIE))
    {
        a_p_this->tx_context.index++;

        bool ret = a_p_this->tx_callback.function(reinterpret_cast<volatile uint32*>(&(I2C1->TXDR)),
                                                  get_bus_status_from_I2C_ISR(Transfer_type::transmit),
                                                  a_p_this->tx_callback.p_user_data);

        if (true == is_I2C_ISR_error())
        {
            clear_I2C_ISR_errors();
        }

        if (false == ret || a_p_this->tx_context.index == a_p_this->tx_context.size)
        {
            a_p_this->tx_callback      = { nullptr, nullptr };
            a_p_this->tx_context.index = 0;
            a_p_this->tx_context.size  = 0;

            set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
            clear_flag(&(I2C1->CR1), I2C_CR1_TXIE);
        }
    }

    if (true == is_flag(isr, I2C_ISR_ADDR) && true == is_flag(cr1, I2C_CR1_ADDRIE))
    {
        set_flag(&(I2C1->ICR), I2C_ICR_ADDRCF);
    }
}

I2C_base::Clock_source I2C_base::get_clock_source() const
{
    return static_cast<Clock_source>(get_flag(RCC->CCIPR, RCC_CCIPR_I2C1SEL) >> RCC_CCIPR_I2C1SEL_Pos);
}

void I2C_master::enable(const Config& a_config,
                        Clock_source a_clock_source,
                        uint32 a_irq_priority)
{
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

uint32 I2C_master::transmit_bytes_polling(uint16 a_slave_address,
                                          const void* a_p_data,
                                          uint32 a_data_size_in_bytes,
                                          Bus_status* a_p_status)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes && false == is_I2C_ISR_error())
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_TXE))
        {
            I2C1->TXDR = static_cast<const byte*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(Transfer_type::transmit);
    }

    if (true == is_I2C_ISR_error())
    {
        clear_I2C_ISR_errors();
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return ret;
}

uint32 I2C_master::transmit_bytes_polling(uint16 a_slave_address,
                                          const void* a_p_data,
                                          uint32 a_data_size_in_bytes,
                                          time::tick a_timeout_ms,
                                          Bus_status* a_p_status)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(true == systick::is_enabled() && a_timeout_ms > 0);

    time::tick start = systick::get_counter();

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error() &&
           a_timeout_ms <= time::diff(hal::systick::get_counter(), start))
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_TXE))
        {
            I2C1->TXDR = static_cast<const byte*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(Transfer_type::transmit);
    }

    if (true == is_I2C_ISR_error())
    {
        clear_I2C_ISR_errors();
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return ret;
}

uint32 I2C_master::receive_bytes_polling(uint16 a_slave_address,
                                         void* a_p_data,
                                         uint32 a_data_size_in_bytes,
                                         Bus_status* a_p_status)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes && false == is_I2C_ISR_error())
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_RXNE))
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(I2C1->RXDR);
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error())
    {
        clear_I2C_ISR_errors();
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return ret;
}

uint32 I2C_master::receive_bytes_polling(uint16 a_slave_address,
                                         void* a_p_data,
                                         uint32 a_data_size_in_bytes,
                                         time::tick a_timeout_ms,
                                         Bus_status* a_p_status)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(true == systick::is_enabled() && a_timeout_ms > 0);

    time::tick start = systick::get_counter();

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error() &&
           a_timeout_ms <= time::diff(hal::systick::get_counter(), start))
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_RXNE))
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(I2C1->RXDR);
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error())
    {
        clear_I2C_ISR_errors();
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return ret;
}

void I2C_master::start_transmit_bytes_it(uint16 a_slave_address,
                                        const TX_callback& a_callback,
                                        uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    this->tx_context.index = 0;
    this->tx_context.size  = a_data_size_in_bytes;

    this->rx_callback = { nullptr, nullptr };
    this->tx_callback = a_callback;

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;
    set_flag(&(I2C1->CR1), I2C_CR1_TXIE);
}

void I2C_master::start_receive_bytes_it(uint16 a_slave_address,
                                        const RX_callback& a_callback,
                                        uint32 a_data_size_in_bytes)
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != a_callback.function);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    this->rx_context.index = 0;
    this->rx_context.size  = a_data_size_in_bytes;

    this->tx_callback = { nullptr, nullptr };
    this->rx_callback = a_callback;

    uint32 address_mask   = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_size_mask = static_cast<uint32>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
    set_flag(&(I2C1->CR1), I2C_CR1_RXIE);
}

void I2C_master::stop_transmit_bytes_it()
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != this->tx_callback.function);

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    clear_flag(&(I2C1->CR1), I2C_CR1_TXIE);

    this->tx_context  = { 0, 0 };
    this->tx_callback = { nullptr, nullptr };
}

void I2C_master::stop_receive_bytes_it()
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(nullptr != this->rx_callback.function);

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    clear_flag(&(I2C1->CR1), I2C_CR1_RXIE);

    this->rx_context  = { 0, 0 };
    this->rx_callback = { nullptr, nullptr };
}

bool I2C_master::is_slave_connected(uint16 a_slave_address, time::tick a_timeout_ms) const
{
    assert(nullptr != controller.p_i2c_master_handle);
    assert(true == systick::is_enabled());

    time::tick start = systick::get_counter();

    uint32 address_mask = (static_cast<uint32>(a_slave_address) << 1) & I2C_CR2_SADD;

    I2C1->CR2 = address_mask | I2C_CR2_AUTOEND | I2C_CR2_START;

    bool ret = wait::until(&(I2C1->ISR), I2C_ISR_STOPF, false, start, a_timeout_ms);

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

void I2C_slave::enable(const Config& a_config,
                       Clock_source a_clock_source,
                       uint32 a_irq_priority)
{
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
                        I2C_CR1_ADDRIE | I2C_CR1_PE;

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

uint32 I2C_slave::transmit_bytes_polling(const void* a_p_data,
                                         uint32 a_data_size_in_bytes,
                                         Bus_status* a_p_status)
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error())
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_TXE))
        {
            I2C1->TXDR = static_cast<const uint8*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(Transfer_type::transmit);
    }

    if (true == is_I2C_ISR_error())
    {
        clear_I2C_ISR_errors();
    }

    return ret;
}

uint32 I2C_slave::transmit_bytes_polling(const void* a_p_data,
                                         uint32 a_data_size_in_bytes,
                                         time::tick a_timeout_ms,
                                         Bus_status* a_p_status)
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(true == systick::is_enabled() && a_timeout_ms > 0);

    time::tick start = systick::get_counter();

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error() &&
           a_timeout_ms <= time::diff(hal::systick::get_counter(), start))
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_TXE))
        {
            I2C1->TXDR = static_cast<const uint8*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error())
    {
        clear_I2C_ISR_errors();
    }

    return ret;
}

common::uint32 I2C_slave::receive_bytes_polling(void* a_p_data,
                                                uint32 a_data_size_in_bytes,
                                                Bus_status* a_p_status)
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error())
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_RXNE))
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(I2C1->RXDR);
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error())
    {
        clear_I2C_ISR_errors();
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);

    return ret;
}

common::uint32 I2C_slave::receive_bytes_polling(void* a_p_data,
                                                uint32 a_data_size_in_bytes,
                                                time::tick a_timeout_ms,
                                                Bus_status* a_p_status)
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(true == systick::is_enabled() && a_timeout_ms > 0);

    time::tick start = systick::get_counter();

    uint32 ret = 0;
    while (ret < a_data_size_in_bytes &&
           false == is_I2C_ISR_error() &&
           a_timeout_ms <= time::diff(hal::systick::get_counter(), start))
    {
        if (true == is_flag(I2C1->ISR, I2C_ISR_RXNE))
        {
            static_cast<uint8*>(a_p_data)[ret++] = static_cast<uint8>(I2C1->RXDR);
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_from_I2C_ISR(Transfer_type::receive);
    }

    if (true == is_I2C_ISR_error())
    {
        clear_I2C_ISR_errors();
    }

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);

    return ret;
}

void I2C_slave::stop_transmit_bytes_it()
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != this->tx_callback.function);

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    clear_flag(&(I2C1->CR1), I2C_CR1_TXIE);

    this->tx_context = { 0, 0 };
    this->tx_callback = { nullptr, nullptr };
}

void I2C_slave::stop_receive_bytes_it()
{
    assert(nullptr != controller.p_i2c_slave_handle);
    assert(nullptr != this->rx_callback.function);

    set_flag(&(I2C1->ICR), I2C_ICR_STOPCF);
    clear_flag(&(I2C1->CR1), I2C_CR1_RXIE);

    this->rx_context = { 0, 0 };
    this->rx_callback = { nullptr, nullptr };
}

} // namespace stm32l011xx
} // namespace hal
} // namespace cml