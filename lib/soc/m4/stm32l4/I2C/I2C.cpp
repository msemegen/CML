/*
 *   Name: I2C.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/I2C/I2C.hpp>

// soc
#include <soc/m4/stm32l4/mcu/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/tick_counter.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;
using namespace cml::utils;

inline bool is_error(I2C_TypeDef* a_p_registers)
{
    return cml::bit::is_any(a_p_registers->ISR,
                            I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR |
                                I2C_ISR_NACKF);
}

template<typename Event_flag_t> Event_flag_t get_Event_flag(I2C_TypeDef* p_registers)
{
    Event_flag_t ret        = Event_flag_t::ok;
    const std::uint32_t isr = p_registers->ISR;

    if (true == cml::bit_flag::is(isr, I2C_ISR_OVR))
    {
        ret |= Event_flag_t::buffer_error;
    }

    if (true == cml::bit_flag::is(isr, I2C_ISR_ARLO))
    {
        ret |= Event_flag_t::arbitration_lost;
    }

    if (true == cml::bit_flag::is(isr, I2C_ISR_BERR))
    {
        ret |= Event_flag_t::misplaced;
    }

    if (true == cml::bit_flag::is(isr, I2C_ISR_NACKF))
    {
        ret |= Event_flag_t::nack;
    }

    return ret;
}

void I2C::disable()
{
    this->p_registers->CR1 = 0;

    if (true == bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx))
    {
        bit::clear(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx);
    }
}

void I2C_master::enable(const Enable_config& a_config)
{
    cml_assert(true == this->is_created());

    cml_assert(various::get_enum_incorrect_value<Enable_config::Analog_filter>() != a_config.analog_filter);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Fast_plus>() != a_config.fast_plus);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Crc>() != a_config.crc);

    cml_assert((Enable_config::Fast_plus::enabled == a_config.fast_plus && true == rcc<mcu>::is_SYSCFG_active()) ||
               Enable_config::Fast_plus::disabled == a_config.fast_plus);

    this->p_registers->CR1 = 0;

    this->p_registers->TIMINGR = a_config.timings;
    this->p_registers->CR2     = I2C_CR2_AUTOEND;
    this->p_registers->CR1 = (Enable_config::Analog_filter::disabled == a_config.analog_filter ? I2C_CR1_ANFOFF : 0) |
                             (Enable_config::Crc::enabled == a_config.crc ? I2C_CR1_PECEN : 0) | I2C_CR1_PE;

    if (Enable_config::Fast_plus::enabled == a_config.fast_plus)
    {
        bit::set(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx);
    }
}

I2C_master::Enable_config I2C_master::get_Enable_config() const
{
    return { static_cast<Enable_config::Analog_filter>(false == bit_flag::is(this->p_registers->CR1, I2C_CR1_ANFOFF)),
             static_cast<Enable_config::Fast_plus>(bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx)),
             static_cast<Enable_config::Crc>(bit_flag::is(this->p_registers->CR1, I2C_CR1_PECEN)),
             this->p_registers->TIMINGR };
}

I2C_master::Polling::Result
I2C_master::Polling::transmit(std::uint8_t a_slave_address, const void* a_p_data, std::size_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFEu);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0u && a_data_size_in_bytes <= 255u);

    const std::uint32_t address_mask = (static_cast<std::uint32_t>(a_slave_address)) & I2C_CR2_SADD;
    const std::uint32_t data_size_mask =
        (static_cast<std::uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;

    this->p_I2C->p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START;

    std::size_t bytes = 0;
    bool error        = false;
    Event_flag event  = Event_flag::ok;

    while (false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            this->p_I2C->p_registers->TXDR = static_cast<const std::uint8_t*>(a_p_data)[bytes++];
        }

        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_TC))
        {
            bit_flag::set(&(this->p_I2C->p_registers->CR2), I2C_CR2_STOP);
        }

        error = is_error(this->p_I2C->p_registers);
    }

    if (true == error)
    {
        event = get_Event_flag<Event_flag>(this->p_I2C->p_registers);

        bit_flag::set(&(this->p_I2C->p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);

        if (false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(this->p_I2C->p_registers->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_STOPCF);
    this->p_I2C->p_registers->CR2 = 0;

    return { event, bytes };
}

I2C_master::Polling::Result I2C_master::Polling::transmit(std::uint8_t a_slave_address,
                                                          const void* a_p_data,
                                                          std::size_t a_data_size_in_bytes,
                                                          Milliseconds a_timeout)
{
    cml_assert(a_slave_address <= 0xFEu);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0u && a_data_size_in_bytes <= 255u);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    const std::uint32_t address_mask   = static_cast<std::uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const std::uint32_t data_size_mask = static_cast<std::uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_I2C->p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START;

    std::size_t bytes = 0u;
    bool error        = false;
    Event_flag event  = Event_flag::ok;

    while (false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= tick_counter::get() - start)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            this->p_I2C->p_registers->TXDR = static_cast<const std::uint8_t*>(a_p_data)[bytes++];
        }

        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_TC))
        {
            bit_flag::set(&(this->p_I2C->p_registers->CR2), I2C_CR2_STOP);
        }

        error = is_error(this->p_I2C->p_registers);
    }

    if (true == error)
    {
        event = get_Event_flag<Event_flag>(this->p_I2C->p_registers);

        bit_flag::set(&(this->p_I2C->p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);

        if (false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(this->p_I2C->p_registers->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_STOPCF);
    this->p_I2C->p_registers->CR2 = 0;

    return { event, bytes };
}

I2C_master::Polling::Result
I2C_master::Polling::receive(std::uint8_t a_slave_address, void* a_p_data, std::size_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFEu);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0u && a_data_size_in_bytes <= 255u);

    const std::uint32_t address_mask   = static_cast<std::uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const std::uint32_t data_size_mask = static_cast<std::uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_I2C->p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_RD_WRN;

    std::size_t bytes = 0u;
    bool error        = false;
    Event_flag event  = Event_flag::ok;

    while (false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<std::uint8_t*>(a_p_data)[bytes++] = static_cast<std::uint8_t>(this->p_I2C->p_registers->RXDR);

            if (a_data_size_in_bytes == bytes)
            {
                bit_flag::set(&(this->p_I2C->p_registers->CR2), I2C_CR2_STOP);
            }
        }

        error = is_error(this->p_I2C->p_registers);
    }

    if (true == error)
    {
        event = get_Event_flag<Event_flag>(this->p_I2C->p_registers);

        bit_flag::set(&(this->p_I2C->p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);

        if (false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(this->p_I2C->p_registers->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_STOPCF);
    this->p_I2C->p_registers->CR2 = 0u;

    return { event, bytes };
}

I2C_master::Polling::Result I2C_master::Polling::receive(std::uint8_t a_slave_address,
                                                         void* a_p_data,
                                                         std::size_t a_data_size_in_bytes,
                                                         Milliseconds a_timeout)
{
    cml_assert(a_slave_address <= 0xFEu);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0u && a_data_size_in_bytes <= 255u);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    const std::uint32_t address_mask   = static_cast<std::uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const std::uint32_t data_size_mask = static_cast<std::uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_I2C->p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_RD_WRN;

    std::size_t bytes = 0;
    bool error        = false;
    Event_flag event  = Event_flag::ok;

    while (false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= tick_counter::get() - start)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<std::uint8_t*>(a_p_data)[bytes++] = static_cast<std::uint8_t>(this->p_I2C->p_registers->RXDR);

            if (a_data_size_in_bytes == bytes)
            {
                bit_flag::set(&(this->p_I2C->p_registers->CR2), I2C_CR2_STOP);
            }
        }

        error = is_error(this->p_I2C->p_registers);
    }

    if (true == error)
    {
        event = get_Event_flag<Event_flag>(this->p_I2C->p_registers);

        bit_flag::set(&(this->p_I2C->p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);

        if (false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(this->p_I2C->p_registers->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_STOPCF);
    this->p_I2C->p_registers->CR2 = 0u;

    return { event, bytes };
}

void I2C_slave::enable(const Enable_config& a_config)
{
    cml_assert(true == this->is_created());

    cml_assert(various::get_enum_incorrect_value<Enable_config::Analog_filter>() != a_config.analog_filter);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Fast_plus>() != a_config.fast_plus);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Crc>() != a_config.crc);
    cml_assert(a_config.address <= 0x7Fu);

    cml_assert((Enable_config::Fast_plus::enabled == a_config.fast_plus && true == rcc<mcu>::is_SYSCFG_active()) ||
               Enable_config::Fast_plus::disabled == a_config.fast_plus);

    this->p_registers->CR1     = 0;
    this->p_registers->TIMINGR = a_config.timings;
    this->p_registers->OAR1    = I2C_OAR1_OA1EN | (a_config.address);
    this->p_registers->CR1 = (Enable_config::Analog_filter::enabled == a_config.analog_filter ? I2C_CR1_ANFOFF : 0u) |
                             (Enable_config::Crc::enabled == a_config.crc ? I2C_CR1_PECEN : 0u) | I2C_CR1_PE;

    if (Enable_config::Fast_plus::enabled == a_config.fast_plus)
    {
        bit::set(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx);
    }
}

I2C_slave::Enable_config I2C_slave::get_Enable_config() const
{
    return { static_cast<Enable_config::Analog_filter>(false == bit_flag::is(this->p_registers->CR1, I2C_CR1_ANFOFF)),
             static_cast<Enable_config::Fast_plus>(bit::is(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + this->idx)),
             static_cast<Enable_config::Crc>(bit_flag::is(this->p_registers->CR1, I2C_CR1_PECEN)),
             this->p_registers->TIMINGR,
             static_cast<std::uint16_t>(this->p_registers->OAR1 & 0x8000u) };
}

I2C_slave::Polling::Result I2C_slave::Polling::transmit(const void* a_p_data, std::size_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255u);

    constexpr std::uint32_t error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    std::size_t bytes = 0;
    bool error        = false;
    Event_flag event  = Event_flag::ok;

    while ((false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF) &&
            false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_NACKF)) &&
           false == error)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            this->p_I2C->p_registers->TXDR = static_cast<const std::uint8_t*>(a_p_data)[bytes++];
        }

        error = bit::is_any(this->p_I2C->p_registers->ISR, error_mask);
    }

    if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF) &&
        true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_NACKF))
    {
        bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        event = get_Event_flag<Event_flag>(this->p_I2C->p_registers);

        bit_flag::set(&(this->p_I2C->p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);
    }

    bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_STOPCF);

    return { event, bytes };
}

I2C_slave::Polling::Result
I2C_slave::Polling::transmit(const void* a_p_data, std::size_t a_data_size_in_bytes, Milliseconds a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255u);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    constexpr std::uint32_t error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    std::size_t bytes = 0;
    bool error        = false;
    Event_flag event  = Event_flag::ok;

    while ((false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF) &&
            false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_NACKF)) &&
           false == error && a_timeout >= tick_counter::get() - start)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            this->p_I2C->p_registers->TXDR = static_cast<const std::uint8_t*>(a_p_data)[bytes++];
        }

        error = bit::is_any(this->p_I2C->p_registers->ISR, error_mask);
    }

    if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF) &&
        true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_NACKF))
    {
        bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        event = get_Event_flag<Event_flag>(this->p_I2C->p_registers);

        bit_flag::set(&(this->p_I2C->p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);
    }

    bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_STOPCF);

    return { event, bytes };
}
I2C_slave::Polling::Result I2C_slave::Polling::receive(void* a_p_data, std::size_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    std::size_t bytes = 0;
    bool error        = false;
    Event_flag event  = Event_flag::ok;

    while (false == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_RXNE))
        {
            const std::uint8_t rxdr = static_cast<std::uint8_t>(this->p_I2C->p_registers->RXDR);

            if (bytes < a_data_size_in_bytes)
            {
                static_cast<std::uint8_t*>(a_p_data)[bytes++] = rxdr;
            }
        }

        error = is_error(this->p_I2C->p_registers);
    }

    if (false == error)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<std::uint8_t*>(a_p_data)[bytes++] = static_cast<std::uint8_t>(this->p_I2C->p_registers->RXDR);
        }
    }
    else
    {
        event = get_Event_flag<Event_flag>(this->p_I2C->p_registers);

        bit_flag::set(&(this->p_I2C->p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);
    }

    bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_STOPCF);

    return { event, bytes };
}

I2C_slave::Polling::Result
I2C_slave::Polling::receive(void* a_p_data, std::size_t a_data_size_in_bytes, Milliseconds a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255u);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    std::size_t bytes = 0;
    bool error        = false;
    Event_flag event  = Event_flag::ok;

    while (false == bit_flag::is(this->p_I2C->p_registers->ICR, I2C_ICR_STOPCF) && false == error &&
           a_timeout >= tick_counter::get() - start)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_RXNE))
        {
            const std::uint8_t rxdr = static_cast<std::uint8_t>(this->p_I2C->p_registers->RXDR);

            if (bytes < a_data_size_in_bytes)
            {
                static_cast<std::uint8_t*>(a_p_data)[bytes++] = rxdr;
            }
        }

        error = is_error(this->p_I2C->p_registers);
    }

    if (false == error)
    {
        if (true == bit_flag::is(this->p_I2C->p_registers->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<std::uint8_t*>(a_p_data)[bytes++] = static_cast<std::uint8_t>(this->p_I2C->p_registers->RXDR);
        }
    }
    else
    {
        event = get_Event_flag<Event_flag>(this->p_I2C->p_registers);

        bit_flag::set(&(this->p_I2C->p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);
    }

    bit_flag::set(&(this->p_I2C->p_registers->ICR), I2C_ICR_STOPCF);

    return { event, bytes };
}

void I2C::Interrupt::enable(const IRQ_config& a_irq_transceiving_config, const IRQ_config& a_irq_event_config)
{
    NVIC_SetPriority(
        this->p_I2C->ev_irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                                         a_irq_transceiving_config.preempt_priority,
                                         a_irq_transceiving_config.sub_priority));
    NVIC_EnableIRQ(this->p_I2C->ev_irqn);

    NVIC_SetPriority(
        this->p_I2C->er_irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                                         a_irq_event_config.preempt_priority,
                                         a_irq_event_config.sub_priority));
    NVIC_EnableIRQ(this->p_I2C->er_irqn);
}

void I2C::Interrupt::disable()
{
    this->transmit_stop();
    this->receive_stop();
    this->event_listening_stop();

    NVIC_DisableIRQ(this->p_I2C->ev_irqn);
    NVIC_DisableIRQ(this->p_I2C->er_irqn);
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif