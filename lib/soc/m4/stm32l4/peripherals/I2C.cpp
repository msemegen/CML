/*
 *   Name: I2C.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/peripherals/I2C.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/mcu.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

// externals
#include <stm32l4xx.h>

namespace {

using namespace cml;
using namespace soc::m4::stm32l4::peripherals;

struct Controller
{
    I2C_TypeDef* p_registers        = nullptr;
    I2C_master* p_i2c_master_handle = nullptr;
    I2C_slave* p_i2c_slave_handle   = nullptr;
};

Controller controllers[]
{
    { I2C1, nullptr, nullptr },
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        { I2C2, nullptr, nullptr },
#endif
        { I2C3, nullptr, nullptr },
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    {
        I2C4, nullptr, nullptr
    }
#endif
};

I2C_TypeDef* get_i2c_ptr(I2C_base::Id a_id)
{
    return controllers[static_cast<uint32_t>(a_id)].p_registers;
}

bool is_I2C_ISR_error(I2C_base::Id a_id)
{
    return bit::is_any(get_i2c_ptr(a_id)->ISR,
                       I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR | I2C_ISR_NACKF);
}

void clear_I2C_ISR_errors(I2C_base::Id a_id)
{
    bit_flag::set(&(get_i2c_ptr(a_id)->ICR),
                  I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF | I2C_ICR_NACKCF);
}

I2C_base::Bus_flag get_bus_status_flag_from_I2C_ISR(I2C_base::Id a_id)
{
    I2C_base::Bus_flag ret = I2C_base::Bus_flag::ok;
    const uint32_t isr     = get_i2c_ptr(a_id)->ISR;

    if (true == bit_flag::is(isr, I2C_ISR_OVR))
    {
        ret |= I2C_base::Bus_flag::buffer_error;
    }

    if (true == bit_flag::is(isr, I2C_ISR_ARLO))
    {
        ret |= I2C_base::Bus_flag::arbitration_lost;
    }

    if (true == bit_flag::is(isr, I2C_ISR_BERR))
    {
        ret |= I2C_base::Bus_flag::misplaced;
    }

    if (true == bit_flag::is(isr, I2C_ISR_NACKF))
    {
        ret |= I2C_base::Bus_flag::nack;
    }

    return ret;
}

} // namespace

extern "C" {

void interrupt_handler(uint32_t a_controller_index)
{
    cml_assert(nullptr != controllers[a_controller_index].p_i2c_master_handle ||
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
    interrupt_handler(0);
}

void I2C2_EV_IRQHandler()
{
    interrupt_handler(1);
}

void I2C3_EV_IRQHandler()
{
    interrupt_handler(2);
}

void I2C4_EV_IRQHandler()
{
    interrupt_handler(3);
}

} // extern "C"

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void i2c_master_interrupt_handler(I2C_master* a_p_this)
{
    const uint32_t isr = get_i2c_ptr(a_p_this->id)->ISR;
    const uint32_t cr1 = get_i2c_ptr(a_p_this->id)->CR1;

    if (true == is_I2C_ISR_error(a_p_this->id))
    {
        const I2C_base::Bus_flag status = get_bus_status_flag_from_I2C_ISR(a_p_this->id);

        if (I2C_base::Bus_flag::ok != status)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            }

            clear_I2C_ISR_errors(a_p_this->id);
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_RXNE) && true == bit_flag::is(cr1, I2C_CR1_RXIE))
        {
            a_p_this->receive_callback.function(static_cast<uint8_t>(get_i2c_ptr(a_p_this->id)->RXDR),
                                                false,
                                                a_p_this,
                                                a_p_this->receive_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_TXE) && true == bit_flag::is(cr1, I2C_CR1_TXIE))
        {
            a_p_this->transmit_callback.function(
                reinterpret_cast<volatile uint32_t*>(&(get_i2c_ptr(a_p_this->id)->TXDR)),
                false,
                a_p_this,
                a_p_this->transmit_callback.p_user_data);
        }
    }

    if (true == bit_flag::is(isr, I2C_ISR_STOPF) && true == bit_flag::is(cr1, I2C_CR1_STOPIE))
    {
        bit_flag::set(&(get_i2c_ptr(a_p_this->id)->ICR), I2C_ICR_STOPCF);

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

    if (true == bit_flag::is(isr, I2C_ISR_NACKF) && nullptr != a_p_this->transmit_callback.function)
    {
        bit_flag::set(&(get_i2c_ptr(a_p_this->id)->ICR), I2C_ICR_NACKCF);
    }
    else
    {
        const I2C_base::Bus_flag status = get_bus_status_flag_from_I2C_ISR(a_p_this->id);

        if (I2C_base::Bus_flag::ok != status)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            }

            clear_I2C_ISR_errors(a_p_this->id);
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_RXNE) && true == bit_flag::is(cr1, I2C_CR1_RXIE))
        {
            a_p_this->receive_callback.function(static_cast<uint8_t>(get_i2c_ptr(a_p_this->id)->RXDR),
                                                false,
                                                a_p_this,
                                                a_p_this->receive_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_TXE) && true == bit_flag::is(cr1, I2C_CR1_TXIE))
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
        if (true == bit_flag::is(isr, I2C_ISR_ADDR) && true == bit_flag::is(cr1, I2C_CR1_ADDRIE))
        {
            a_p_this->address_match_callback.function(a_p_this, a_p_this->address_match_callback.p_user_data);

            bit_flag::set(&(get_i2c_ptr(a_p_this->id)->ICR), I2C_ICR_ADDRCF);
        }
    }

    if (true == bit_flag::is(isr, I2C_ISR_STOPF) && true == bit_flag::is(cr1, I2C_CR1_STOPIE))
    {
        bit_flag::set(&(get_i2c_ptr(a_p_this->id)->ICR), I2C_ICR_STOPCF);

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

bool I2C_base::is_enabled() const
{
    return bit_flag::is(controllers[static_cast<uint32_t>(this->id)].p_registers->CR1, I2C_CR1_PE);
}

void I2C_master::enable(const Config& a_config, uint32_t a_irq_priority)
{
    cml_assert(various::get_enum_incorrect_value<Config::Analog_filter>() != a_config.analog_filter);
    cml_assert(various::get_enum_incorrect_value<Config::Fast_plus>() != a_config.fast_plus);
    cml_assert(various::get_enum_incorrect_value<Config::Crc>() != a_config.crc);

    cml_assert((Config::Fast_plus::enabled == a_config.fast_plus &&
                rcc<mcu>::SYSCFG_mode::enabled == rcc<mcu>::get_syscfg_mode()) ||
               Config::Fast_plus::disabled == a_config.fast_plus);
    cml_assert(nullptr == controllers[static_cast<uint32_t>(this->id)].p_i2c_slave_handle &&
               nullptr == controllers[static_cast<uint32_t>(this->id)].p_i2c_master_handle);

    controllers[static_cast<uint32_t>(this->id)].p_i2c_slave_handle  = nullptr;
    controllers[static_cast<uint32_t>(this->id)].p_i2c_master_handle = this;

    get_i2c_ptr(this->id)->CR1     = 0;
    get_i2c_ptr(this->id)->TIMINGR = a_config.timings;

    get_i2c_ptr(this->id)->CR2 = I2C_CR2_AUTOEND;

    get_i2c_ptr(this->id)->CR1 = (Config::Analog_filter::disabled == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                                 (Config::Crc::enabled == a_config.crc ? I2C_CR1_PECEN : 0) | I2C_CR1_PE;

    if (Config::Fast_plus::enabled == a_config.fast_plus)
    {
        bit::set(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id));
    }

    switch (this->id)
    {
        case Id::_1: {
            NVIC_SetPriority(IRQn_Type::I2C1_ER_IRQn, a_irq_priority);
            NVIC_SetPriority(IRQn_Type::I2C1_EV_IRQn, a_irq_priority);
            NVIC_EnableIRQ(IRQn_Type::I2C1_ER_IRQn);
            NVIC_EnableIRQ(IRQn_Type::I2C1_EV_IRQn);
        }
        break;

        case Id::_3: {
            NVIC_SetPriority(IRQn_Type::I2C3_ER_IRQn, a_irq_priority);
            NVIC_SetPriority(IRQn_Type::I2C3_EV_IRQn, a_irq_priority);
            NVIC_EnableIRQ(IRQn_Type::I2C3_ER_IRQn);
            NVIC_EnableIRQ(IRQn_Type::I2C3_EV_IRQn);
        }
        break;
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Id::_2: {
            NVIC_SetPriority(IRQn_Type::I2C2_ER_IRQn, a_irq_priority);
            NVIC_SetPriority(IRQn_Type::I2C2_EV_IRQn, a_irq_priority);
            NVIC_EnableIRQ(IRQn_Type::I2C2_ER_IRQn);
            NVIC_EnableIRQ(IRQn_Type::I2C2_EV_IRQn);
        }
        break;
#endif
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Id::_4: {
            NVIC_SetPriority(IRQn_Type::I2C4_ER_IRQn, a_irq_priority);
            NVIC_SetPriority(IRQn_Type::I2C4_EV_IRQn, a_irq_priority);
            NVIC_EnableIRQ(IRQn_Type::I2C4_ER_IRQn);
            NVIC_EnableIRQ(IRQn_Type::I2C4_EV_IRQn);
        }
        break;
#endif
    }
}

void I2C_master::diasble()
{
    switch (this->id)
    {
        case Id::_1: {
            NVIC_DisableIRQ(IRQn_Type::I2C1_ER_IRQn);
            NVIC_DisableIRQ(IRQn_Type::I2C1_EV_IRQn);
        }
        break;

        case Id::_3: {
            NVIC_DisableIRQ(IRQn_Type::I2C3_ER_IRQn);
            NVIC_DisableIRQ(IRQn_Type::I2C3_EV_IRQn);
        }
        break;
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Id::_2: {
            NVIC_DisableIRQ(IRQn_Type::I2C2_ER_IRQn);
            NVIC_DisableIRQ(IRQn_Type::I2C2_EV_IRQn);
        }
        break;
#endif
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Id::_4: {
            NVIC_DisableIRQ(IRQn_Type::I2C4_ER_IRQn);
            NVIC_DisableIRQ(IRQn_Type::I2C4_EV_IRQn);
        }
        break;
#endif
    }

    get_i2c_ptr(this->id)->CR1 = 0;

    if (true == bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id)))
    {
        bit::clear(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id));
    }

    controllers[static_cast<uint32_t>(this->id)].p_i2c_master_handle = nullptr;
}

I2C_master::Result
I2C_master::transmit_bytes_polling(uint8_t a_slave_address, const void* a_p_data, uint32_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    const uint32_t address_mask = (static_cast<uint32_t>(a_slave_address)) & I2C_CR2_SADD;
    const uint32_t data_size_mask =
        (static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START;

    uint32_t bytes      = 0;
    bool error          = false;
    Bus_flag bus_status = Bus_flag::ok;

    while (false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            get_i2c_ptr(this->id)->TXDR = static_cast<const uint8_t*>(a_p_data)[bytes++];
        }

        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_TC))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->CR2), I2C_CR2_STOP);
        }

        error = is_I2C_ISR_error(this->id);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(this->id);

        clear_I2C_ISR_errors(this->id);

        if (false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    get_i2c_ptr(this->id)->CR2 = 0;

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

    const uint32_t address_mask   = static_cast<uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START;

    uint32_t bytes      = 0;
    bool error          = false;
    Bus_flag bus_status = Bus_flag::ok;

    while (false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            get_i2c_ptr(this->id)->TXDR = static_cast<const uint8_t*>(a_p_data)[bytes++];
        }

        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_TC))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->CR2), I2C_CR2_STOP);
        }

        error = is_I2C_ISR_error(this->id);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(this->id);

        clear_I2C_ISR_errors(this->id);

        if (false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    get_i2c_ptr(this->id)->CR2 = 0;

    return { bus_status, bytes };
}

I2C_master::Result
I2C_master::receive_bytes_polling(uint8_t a_slave_address, void* a_p_data, uint32_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    const uint32_t address_mask   = static_cast<uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_RD_WRN;

    uint32_t bytes      = 0;
    bool error          = false;
    Bus_flag bus_status = Bus_flag::ok;

    while (false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);

            if (a_data_size_in_bytes == bytes)
            {
                bit_flag::set(&(get_i2c_ptr(this->id)->CR2), I2C_CR2_STOP);
            }
        }

        error = is_I2C_ISR_error(this->id);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(this->id);

        clear_I2C_ISR_errors(this->id);

        if (false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    get_i2c_ptr(this->id)->CR2 = 0;

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

    const uint32_t address_mask   = static_cast<uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_RD_WRN;

    uint32_t bytes      = 0;
    bool error          = false;
    Bus_flag bus_status = Bus_flag::ok;

    while (false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);

            if (a_data_size_in_bytes == bytes)
            {
                bit_flag::set(&(get_i2c_ptr(this->id)->CR2), I2C_CR2_STOP);
            }
        }

        error = is_I2C_ISR_error(this->id);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(this->id);

        clear_I2C_ISR_errors(this->id);

        if (false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    get_i2c_ptr(this->id)->CR2 = 0;

    return { bus_status, bytes };
}

void I2C_master::register_transmit_callback(uint8_t a_slave_address,
                                            const Transmit_callback& a_callback,
                                            uint32_t a_data_length_in_bytes)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    const uint32_t address_mask   = static_cast<uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_length_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;
    bit_flag::set(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);
}

void I2C_master::register_receive_callback(uint8_t a_slave_address,
                                           const Receive_callback& a_callback,
                                           uint32_t a_data_length_in_bytes)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    const uint32_t address_mask   = static_cast<uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_length_in_bytes) << I2C_CR2_NBYTES_Pos;

    get_i2c_ptr(this->id)->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
    bit_flag::set(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);
}

void I2C_master::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;
    bit_flag::set(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);
}

void I2C_master::unregister_transmit_callback()
{
    cml_assert(nullptr != this->transmit_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);

    this->transmit_callback = { nullptr, nullptr };
}

void I2C_master::unregister_receive_callback()
{
    cml_assert(nullptr != this->receive_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);

    this->receive_callback = { nullptr, nullptr };
}

void I2C_master::unregister_bus_status_callback()
{
    cml_assert(nullptr != this->bus_status_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);

    this->bus_status_callback = { nullptr, nullptr };
}

bool I2C_master::is_slave_connected(uint8_t a_slave_address, uint32_t a_timeout) const
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    const uint32_t address_mask = (static_cast<uint32_t>(a_slave_address)) & I2C_CR2_SADD;

    get_i2c_ptr(this->id)->CR2 = address_mask | I2C_CR2_AUTOEND | I2C_CR2_START;

    bool ret = wait_until::all_bits(&(get_i2c_ptr(this->id)->ISR), I2C_ISR_STOPF, false, start, a_timeout);

    if (true == ret)
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_NACKF))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_NACKCF);
            ret = false;
        }
    }

    bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);
    wait_until::all_bits(&(get_i2c_ptr(this->id)->ISR), I2C_ISR_STOPF, true);
    get_i2c_ptr(this->id)->CR2 = 0;

    return ret;
}

I2C_master::Config I2C_master::get_config() const
{
    return { static_cast<Config::Analog_filter>(bit_flag::is(get_i2c_ptr(this->id)->CR1, I2C_CR1_ANFOFF)),
             static_cast<Config::Fast_plus>(
                 bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id))),
             static_cast<Config::Crc>(bit_flag::is(get_i2c_ptr(this->id)->CR1, I2C_CR1_PECEN)),
             get_i2c_ptr(this->id)->TIMINGR };
}

void I2C_slave::enable(const Config& a_config, uint32_t a_irq_priority)
{
    cml_assert(various::get_enum_incorrect_value<Config::Analog_filter>() != a_config.analog_filter);
    cml_assert(various::get_enum_incorrect_value<Config::Fast_plus>() != a_config.fast_plus);
    cml_assert(various::get_enum_incorrect_value<Config::Crc>() != a_config.crc);
    cml_assert(a_config.address <= 0x7F);

    cml_assert((Config::Fast_plus::enabled == a_config.fast_plus &&
                rcc<mcu>::SYSCFG_mode::enabled == rcc<mcu>::get_syscfg_mode()) ||
               Config::Fast_plus::disabled == a_config.fast_plus);
    cml_assert(nullptr == controllers[static_cast<uint32_t>(this->id)].p_i2c_slave_handle &&
               nullptr == controllers[static_cast<uint32_t>(this->id)].p_i2c_master_handle);

    controllers[static_cast<uint32_t>(this->id)].p_i2c_slave_handle = this;

    get_i2c_ptr(this->id)->CR1     = 0;
    get_i2c_ptr(this->id)->TIMINGR = a_config.timings;
    get_i2c_ptr(this->id)->OAR1    = I2C_OAR1_OA1EN | (a_config.address);
    get_i2c_ptr(this->id)->CR1     = (Config::Analog_filter::enabled == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                                 (Config::Crc::enabled == a_config.crc ? I2C_CR1_PECEN : 0) | I2C_CR1_PE;

    if (Config::Fast_plus::enabled == a_config.fast_plus)
    {
        bit::set(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id));
    }
}

void I2C_slave::diasble()
{
    get_i2c_ptr(this->id)->CR1 = 0;

    if (true == bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id)))
    {
        bit::clear(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id));
    }

    controllers[static_cast<uint32_t>(this->id)].p_i2c_slave_handle = nullptr;
}

I2C_slave::Result I2C_slave::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    constexpr uint32_t error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    uint32_t bytes      = 0;
    bool error          = false;
    Bus_flag bus_status = Bus_flag::ok;

    while ((false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) &&
            false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_NACKF)) &&
           false == error)
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            get_i2c_ptr(this->id)->TXDR = static_cast<const uint8_t*>(a_p_data)[bytes++];
        }

        error = bit::is_any(get_i2c_ptr(this->id)->ISR, error_mask);
    }

    if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) &&
        true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_NACKF))
    {
        bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(this->id);

        if (Bus_flag::ok != bus_status)
        {
            clear_I2C_ISR_errors(this->id);
        }
    }

    bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);

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

    uint32_t bytes      = 0;
    bool error          = false;
    Bus_flag bus_status = Bus_flag::ok;

    while ((false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) &&
            false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_NACKF)) &&
           false == error && a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            get_i2c_ptr(this->id)->TXDR = static_cast<const uint8_t*>(a_p_data)[bytes++];
        }

        error = bit::is_any(get_i2c_ptr(this->id)->ISR, error_mask);
    }

    if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) &&
        true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_NACKF))
    {
        bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(this->id);

        if (Bus_flag::ok != bus_status)
        {
            clear_I2C_ISR_errors(this->id);
        }
    }

    bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}

I2C_slave::Result I2C_slave::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32_t bytes      = 0;
    bool error          = false;
    Bus_flag bus_status = Bus_flag::ok;

    while (false == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE))
        {
            const uint8_t rxdr = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);

            if (bytes < a_data_size_in_bytes)
            {
                static_cast<uint8_t*>(a_p_data)[bytes++] = rxdr;
            }
        }

        error = is_I2C_ISR_error(this->id);
    }

    if (false == error)
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);
        }
    }
    else
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(this->id);

        if (Bus_flag::ok != bus_status)
        {
            clear_I2C_ISR_errors(this->id);
        }
    }

    bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}

I2C_slave::Result I2C_slave::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    uint32_t bytes      = 0;
    bool error          = false;
    Bus_flag bus_status = Bus_flag::ok;

    while (false == bit_flag::is(get_i2c_ptr(this->id)->ICR, I2C_ICR_STOPCF) && false == error &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE))
        {
            const uint8_t rxdr = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);

            if (bytes < a_data_size_in_bytes)
            {
                static_cast<uint8_t*>(a_p_data)[bytes++] = rxdr;
            }
        }

        error = is_I2C_ISR_error(this->id);
    }

    if (false == error)
    {
        if (true == bit_flag::is(get_i2c_ptr(this->id)->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<uint8_t*>(a_p_data)[bytes++] = static_cast<uint8_t>(get_i2c_ptr(this->id)->RXDR);
        }
    }
    else
    {
        bus_status = get_bus_status_flag_from_I2C_ISR(this->id);

        if (Bus_flag::ok != bus_status)
        {
            clear_I2C_ISR_errors(this->id);
        }
    }

    bit_flag::set(&(get_i2c_ptr(this->id)->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}

void I2C_slave::register_transmit_callback(const Transmit_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    bit_flag::set(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE | I2C_CR1_NACKIE);
}

void I2C_slave::register_receive_callback(const Receive_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    bit_flag::set(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);
}

void I2C_slave::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;
    bit_flag::set(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_NACKIE | I2C_CR1_ADDRIE | I2C_CR1_ERRIE);
}

void I2C_slave::register_address_match_callback(const Addres_match_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->address_match_callback = a_callback;

    bit_flag::set(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_ADDRIE);
}

void I2C_slave::unregister_transmit_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);

    this->transmit_callback = { nullptr, nullptr };
}

void I2C_slave::unregister_receive_callback()
{
    cml_assert(nullptr != this->receive_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);

    this->receive_callback = { nullptr, nullptr };
}

void I2C_slave::unregister_bus_status_callback()
{
    cml_assert(nullptr != this->bus_status_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_NACKIE | I2C_CR1_ADDRIE | I2C_CR1_ERRIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void I2C_slave::unregister_address_match_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(get_i2c_ptr(this->id)->CR1), I2C_CR1_ADDRIE);

    this->address_match_callback = { nullptr, nullptr };
}

I2C_slave::Config I2C_slave::get_config() const
{
    return { static_cast<Config::Analog_filter>(bit_flag::is(get_i2c_ptr(this->id)->CR1, I2C_CR1_ANFOFF)),
             static_cast<Config::Fast_plus>(
                 bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32_t>(this->id))),
             static_cast<Config::Crc>(bit_flag::is(get_i2c_ptr(this->id)->CR1, I2C_CR1_PECEN)),
             get_i2c_ptr(this->id)->TIMINGR,
             static_cast<uint16_t>(get_i2c_ptr(this->id)->OAR1 & 0x7Fu) };
}

} // namespace peripherals
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace soc::m4::stm32l4::peripherals;

void rcc<I2C_base>::enable(I2C_base::Id a_id, Clock_source a_clock_source, bool a_enable_in_lp)
{
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    if (I2C_base::Id::_4 != a_id)
    {
        bit_flag::set(&(RCC->CCIPR),
                      0x3ul << (RCC_CCIPR_I2C1SEL_Pos + static_cast<uint32_t>(a_id) * 2),
                      static_cast<uint32_t>(a_clock_source)
                          << (RCC_CCIPR_I2C1SEL_Pos + static_cast<uint32_t>(a_id) * 2));
        bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN_Pos + static_cast<uint32_t>(a_id));

        if (true == a_enable_in_lp)
        {
            bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN_Pos + static_cast<uint32_t>(a_id));
        }
    }
    else
    {
        bit_flag::set(&(RCC->CCIPR2), RCC_CCIPR2_I2C4SEL, static_cast<uint32_t>(a_clock_source));
        bit::set(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN_Pos);

        if (true == a_enable_in_lp)
        {
            bit::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_I2C4SMEN_Pos);
        }
    }
#else
    bit_flag::set(&(RCC->CCIPR),
                  0x3ul << (RCC_CCIPR_I2C1SEL_Pos + static_cast<uint32_t>(a_id) * 2),
                  static_cast<uint32_t>(a_clock_source) << (RCC_CCIPR_I2C1SEL_Pos + static_cast<uint32_t>(a_id) * 2));
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN_Pos + static_cast<uint32_t>(a_id));

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN_Pos + static_cast<uint32_t>(a_id));
    }
#endif
}
void rcc<I2C_base>::disable(I2C_base::Id a_id)
{
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    if (I2C_base::Id::_4 != a_id)
    {
        bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN_Pos + static_cast<uint32_t>(a_id));
        bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN_Pos + static_cast<uint32_t>(a_id));
    }
    else
    {
        bit::clear(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN_Pos);
        bit::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_I2C4SMEN_Pos);
    }
#else
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN_Pos + static_cast<uint32_t>(a_id));
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN_Pos + static_cast<uint32_t>(a_id));
#endif
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4