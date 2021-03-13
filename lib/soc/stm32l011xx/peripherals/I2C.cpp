/*
    Name: I2C.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/peripherals/I2C.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/stm32l011xx/mcu.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l011xx::peripherals;

struct Controller
{
    I2C_master* p_i2c_master_handle = nullptr;
    I2C_slave* p_i2c_slave_handle   = nullptr;
};

Controller controller;

void i2c_1_enable(uint32_t a_clock_source, uint32_t a_irq_priority)
{
    cml_assert(0 != a_clock_source);

    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_I2C1SEL, a_clock_source);
    bit_flag::set(&(RCC->APB1ENR), (RCC_APB1ENR_I2C1EN));

    NVIC_SetPriority(I2C1_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C1_IRQn);
}

void i2c_1_disable()
{
    bit_flag::clear(&(RCC->APB1ENR), (RCC_APB1ENR_I2C1EN));
    NVIC_DisableIRQ(I2C1_IRQn);
}

bool is_I2C_ISR_error(uint32_t a_isr)
{
    return bit::is_any(a_isr,
                       I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR | I2C_ISR_NACKF);
}

void clear_I2C_ISR_errors(volatile uint32_t* a_p_icr)
{
    bit_flag::set(a_p_icr,
                  I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF | I2C_ICR_NACKCF);
}

I2C_base::Result::Bus_flag get_bus_status_flag_from_I2C_ISR(uint32_t a_isr)
{
    I2C_base::Result::Bus_flag ret = I2C_base::Result::Bus_flag::ok;

    if (true == bit_flag::is(a_isr, I2C_ISR_OVR))
    {
        ret |= I2C_base::Result::Bus_flag::buffer_error;
    }

    if (true == bit_flag::is(a_isr, I2C_ISR_ARLO))
    {
        ret |= I2C_base::Result::Bus_flag::arbitration_lost;
    }

    if (true == bit_flag::is(a_isr, I2C_ISR_BERR))
    {
        ret |= I2C_base::Result::Bus_flag::misplaced;
    }

    if (true == bit_flag::is(a_isr, I2C_ISR_NACKF))
    {
        ret |= I2C_base::Result::Bus_flag::nack;
    }

    return ret;
}

} // namespace

extern "C" {

void I2C1_IRQHandler()
{
    cml_assert(nullptr != controller.p_i2c_master_handle || nullptr != controller.p_i2c_slave_handle);

    if (nullptr != controller.p_i2c_master_handle)
    {
        i2c_master_interrupt_handler(controller.p_i2c_master_handle);
    }
    else if (nullptr != controller.p_i2c_slave_handle)
    {
        i2c_slave_interrupt_handler(controller.p_i2c_slave_handle);
    }
}

} // extern "C"

namespace soc {
namespace stm32l011xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void i2c_master_interrupt_handler(I2C_master* a_p_this)
{
    const uint32_t isr = I2C1->ISR;
    const uint32_t cr1 = I2C1->CR1;

    if (true == is_I2C_ISR_error(isr))
    {
        const I2C_base::Result::Bus_flag status = get_bus_status_flag_from_I2C_ISR(isr);

        if (I2C_base::Result::Bus_flag::ok != status)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            }

            clear_I2C_ISR_errors(&(I2C1->ICR));
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_RXNE) && true == bit_flag::is(cr1, I2C_CR1_RXIE))
        {
            a_p_this->receive_callback.function(
                static_cast<uint8_t>(I2C1->RXDR), false, a_p_this, a_p_this->receive_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_TXE) && true == bit_flag::is(cr1, I2C_CR1_TXIE))
        {
            a_p_this->transmit_callback.function(reinterpret_cast<volatile uint32_t*>(&(I2C1->TXDR)),
                                                 false,
                                                 a_p_this,
                                                 a_p_this->transmit_callback.p_user_data);
        }
    }

    if (true == bit_flag::is(isr, I2C_ISR_STOPF) && true == bit_flag::is(cr1, I2C_CR1_STOPIE))
    {
        if (nullptr != a_p_this->transmit_callback.function)
        {
            a_p_this->transmit_callback.function(nullptr, true, a_p_this, a_p_this->transmit_callback.p_user_data);
        }

        if (nullptr != a_p_this->receive_callback.function)
        {
            a_p_this->receive_callback.function(0, true, a_p_this, a_p_this->receive_callback.p_user_data);
        }

        bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);
    }
}

void i2c_slave_interrupt_handler(I2C_slave* a_p_this)
{
    const uint32_t isr = I2C1->ISR;
    const uint32_t cr1 = I2C1->CR1;

    if (true == bit_flag::is(isr, I2C_ISR_NACKF) && nullptr != a_p_this->transmit_callback.function)
    {
        bit_flag::set(&(I2C1->ICR), I2C_ICR_NACKCF);
    }
    else
    {
        const I2C_base::Result::Bus_flag status = get_bus_status_flag_from_I2C_ISR(isr);

        if (I2C_base::Result::Bus_flag::ok != status)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            }
            clear_I2C_ISR_errors(&(I2C1->ICR));
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_RXNE) && true == bit_flag::is(cr1, I2C_CR1_RXIE))
        {
            a_p_this->receive_callback.function(
                static_cast<uint8_t>(I2C1->RXDR), false, a_p_this, a_p_this->receive_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_TXE) && true == bit_flag::is(cr1, I2C_CR1_TXIE))
        {
            a_p_this->transmit_callback.function(reinterpret_cast<volatile uint32_t*>(&(I2C1->TXDR)),
                                                 false,
                                                 a_p_this,
                                                 a_p_this->transmit_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->address_match_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_ADDR) && true == bit_flag::is(cr1, I2C_CR1_ADDRIE))
        {
            a_p_this->address_match_callback.function(a_p_this, a_p_this->address_match_callback.p_user_data);
            bit_flag::set(&(I2C1->ICR), I2C_ICR_ADDRCF);
        }
    }

    if (true == bit_flag::is(isr, I2C_ISR_STOPF) && true == bit_flag::is(cr1, I2C_CR1_STOPIE))
    {
        bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);

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

I2C_base::Clock_source I2C_base::get_clock_source() const
{
    return static_cast<Clock_source>(bit_flag::get(RCC->CCIPR, RCC_CCIPR_I2C1SEL) >> RCC_CCIPR_I2C1SEL_Pos);
}

void I2C_master::enable(const Config& a_config, Clock_source a_clock_source, uint32_t a_irq_priority)
{
    cml_assert((Config::Fast_plus::enabled == a_config.fast_plus && true == mcu::is_syscfg_enabled()) ||
               Config::Fast_plus::disabled == a_config.fast_plus);
    cml_assert(nullptr == controller.p_i2c_master_handle && nullptr == controller.p_i2c_slave_handle);

    i2c_1_enable(static_cast<uint32_t>(a_clock_source) << RCC_CCIPR_I2C1SEL_Pos, a_irq_priority);
    controller.p_i2c_master_handle = this;
    controller.p_i2c_slave_handle  = nullptr;

    I2C1->CR1     = 0;
    I2C1->TIMINGR = a_config.timings;

    I2C1->CR1 = (Config::Analog_filter::enabled == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                (Config::Crc::enabled == a_config.crc ? I2C_CR1_PECEN : 0) | I2C_CR1_PE;

    if (Config::Fast_plus::enabled == a_config.fast_plus)
    {
        bit::set(&(SYSCFG->CFGR2), SYSCFG_CFGR2_I2C1_FMP_Pos);
    }
}

void I2C_master::diasble()
{
    I2C1->CR1 = 0;

    if (true == this->is_fast_plus())
    {
        bit::clear(&(SYSCFG->CFGR2), SYSCFG_CFGR2_I2C1_FMP_Pos);
    }

    i2c_1_disable();
    controller.p_i2c_master_handle = nullptr;
}

I2C_master::Result
I2C_master::transmit_bytes_polling(uint8_t a_slave_address, const void* a_p_data, uint32_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    const uint32_t address_mask = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask =
        (static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32_t bytes              = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(I2C1->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            I2C1->TXDR = static_cast<const uint8_t*>(a_p_data)[bytes++];
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return { bus_status, bytes };
}

I2C_master::Result I2C_master::transmit_bytes_polling(uint8_t a_slave_address,
                                                      const void* a_p_data,
                                                      uint32_t a_data_size_in_bytes,
                                                      uint32_t a_timeout)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;

    uint32_t bytes              = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(I2C1->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            I2C1->TXDR = static_cast<const uint8_t*>(a_p_data)[bytes++];
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return { bus_status, bytes };
}

I2C_master::Result
I2C_master::receive_bytes_polling(uint8_t a_slave_address, void* a_p_data, uint32_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32_t bytes              = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(I2C1->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(I2C1->RXDR);
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (false == error)
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(I2C1->RXDR);
        }
    }
    else
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return { bus_status, bytes };
}

I2C_master::Result I2C_master::receive_bytes_polling(uint8_t a_slave_address,
                                                     void* a_p_data,
                                                     uint32_t a_data_size_in_bytes,
                                                     uint32_t a_timeout)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    uint32_t bytes              = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(I2C1->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(I2C1->RXDR);
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (false == error)
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(I2C1->RXDR);
        }
    }
    else
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return { bus_status, bytes };
}

void I2C_master::register_transmit_callback(uint8_t a_slave_address,
                                            const Transmit_callback& a_callback,
                                            uint32_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_callback.function);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;
    bit_flag::set(&(I2C1->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);
}

void I2C_master::register_receive_callback(uint8_t a_slave_address,
                                           const Receive_callback& a_callback,
                                           uint32_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_callback.function);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    const uint32_t address_mask   = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C1->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
    bit_flag::set(&(I2C1->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);
}

void I2C_master::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;

    bit_flag::set(&(I2C1->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);
}

void I2C_master::unregister_transmit_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(I2C1->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);

    this->transmit_callback = { nullptr, nullptr };
}

void I2C_master::unregister_receive_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(I2C1->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);

    this->receive_callback = { nullptr, nullptr };
}

void I2C_master::unregister_bus_status_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(I2C1->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);

    this->bus_status_callback = { nullptr, nullptr };
}

bool I2C_master::is_slave_connected(uint8_t a_slave_address, uint32_t a_timeout) const
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    uint32_t address_mask = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;

    I2C1->CR2 = address_mask | I2C_CR2_AUTOEND | I2C_CR2_START;

    bool ret = wait_until::all_bits(&(I2C1->ISR), I2C_ISR_STOPF, false, start, a_timeout);

    if (true == ret)
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_NACKF))
        {
            bit_flag::set(&(I2C1->ICR), I2C_ICR_NACKCF);
            ret = false;
        }
    }

    bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);
    I2C1->CR2 = 0;

    return ret;
}

void I2C_slave::enable(const Config& a_config, Clock_source a_clock_source, uint32_t a_irq_priority)
{
    cml_assert(a_config.address <= 0x7F);
    cml_assert((Config::Fast_plus::enabled == a_config.fast_plus && true == mcu::is_syscfg_enabled()) ||
               Config::Fast_plus::disabled == a_config.fast_plus);
    cml_assert(nullptr == controller.p_i2c_master_handle && nullptr == controller.p_i2c_slave_handle);

    i2c_1_enable(static_cast<uint32_t>(a_clock_source) << RCC_CCIPR_I2C1SEL_Pos, a_irq_priority);
    controller.p_i2c_slave_handle = this;

    I2C1->CR1     = 0;
    I2C1->TIMINGR = a_config.timings;
    I2C1->OAR1    = I2C_OAR1_OA1EN | (a_config.address << 1);
    I2C1->CR1     = (Config::Analog_filter::enabled == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                (Config::Crc::enabled == a_config.crc ? I2C_CR1_PECEN : 0) | I2C_CR1_PE;

    if (Config::Fast_plus::enabled == a_config.fast_plus)
    {
        bit::set(&(SYSCFG->CFGR2), SYSCFG_CFGR2_I2C1_FMP_Pos);
    }
}

void I2C_slave::diasble()
{
    I2C1->CR1 = 0;

    if (true == this->is_fast_plus())
    {
        bit::clear(&(SYSCFG->CFGR2), SYSCFG_CFGR2_I2C1_FMP_Pos);
    }

    i2c_1_disable();
    controller.p_i2c_slave_handle = nullptr;
}

I2C_slave::Result I2C_slave::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    constexpr uint32_t error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    uint32_t bytes              = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while ((false == bit_flag::is(I2C1->ISR, I2C_ISR_STOPF) && false == bit_flag::is(I2C1->ISR, I2C_ISR_NACKF)) &&
           false == error)
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(I2C1->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            I2C1->TXDR = static_cast<const uint8_t*>(a_p_data)[bytes++];
        }

        error = bit::is_any(I2C1->ISR, error_mask);
    }

    if (true == bit_flag::is(I2C1->ISR, I2C_ISR_STOPF) && true == bit_flag::is(I2C1->ISR, I2C_ISR_NACKF))
    {
        bit_flag::set(&(I2C1->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}

I2C_slave::Result
I2C_slave::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_bytes, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    constexpr uint32_t error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    uint32_t bytes              = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while ((false == bit_flag::is(I2C1->ISR, I2C_ISR_STOPF) && false == bit_flag::is(I2C1->ISR, I2C_ISR_NACKF)) &&
           false == error && a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(I2C1->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            I2C1->TXDR = static_cast<const uint8_t*>(a_p_data)[bytes++];
        }

        error = bit::is_any(I2C1->ISR, error_mask);
    }

    if (true == bit_flag::is(I2C1->ISR, I2C_ISR_STOPF) && true == bit_flag::is(I2C1->ISR, I2C_ISR_NACKF))
    {
        bit_flag::set(&(I2C1->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}

I2C_slave::Result I2C_slave::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32_t bytes              = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(I2C1->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(I2C1->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_RXNE))
        {
            const uint8_t rxdr = static_cast<uint8_t>(I2C1->RXDR);

            if (bytes < a_data_size_in_bytes)
            {
                static_cast<uint8_t*>(a_p_data)[bytes] = rxdr;
            }

            bytes++;
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (false == error)
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(I2C1->RXDR);
        }
    }
    else
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}

I2C_slave::Result I2C_slave::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    uint32_t bytes              = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(I2C1->ICR, I2C_ICR_STOPCF) && false == error &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(I2C1->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_RXNE))
        {
            const uint8_t rxdr = static_cast<uint8_t>(I2C1->RXDR);

            if (bytes < a_data_size_in_bytes)
            {
                static_cast<uint8_t*>(a_p_data)[bytes] = rxdr;
            }

            bytes++;
        }

        error = is_I2C_ISR_error(I2C1->ISR);
    }

    if (false == error)
    {
        if (true == bit_flag::is(I2C1->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(I2C1->RXDR);
        }
    }
    else
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(I2C1->ISR);
        clear_I2C_ISR_errors(&(I2C1->ICR));
    }

    bit_flag::set(&(I2C1->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}

void I2C_slave::register_transmit_callback(const Transmit_callback& a_callback, uint32_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_callback.function);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    bit_flag::set(&(I2C1->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE);
}

void I2C_slave::register_receive_callback(const Receive_callback& a_callback, uint32_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_callback.function);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    bit_flag::set(&(I2C1->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);
}

void I2C_slave::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;
    bit_flag::set(&(I2C1->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);
}

void I2C_slave::register_address_match_callback(const Addres_match_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->address_match_callback = a_callback;

    bit_flag::set(&(I2C1->CR1), I2C_CR1_ADDRIE);
}

void I2C_slave::unregister_transmit_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(I2C1->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);

    this->transmit_callback = { nullptr, nullptr };
}

void I2C_slave::unregister_receive_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(I2C1->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);

    this->receive_callback = { nullptr, nullptr };
}

void I2C_slave::unregister_bus_status_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(I2C1->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void I2C_slave::unregister_address_match_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(I2C1->CR1), I2C_CR1_ADDRIE);

    this->address_match_callback = { nullptr, nullptr };
}

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx