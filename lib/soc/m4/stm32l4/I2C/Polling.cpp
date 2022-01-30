/*
 *   Name: Polling.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/I2C/Polling.hpp>

// soc
#include <soc/m4/stm32l4/I2C/common.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/ms_tick_counter.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;
using namespace cml::utils;

Polling<I2C_master>::Result
Polling<I2C_master>::transmit(std::uint8_t a_slave_address, const void* a_p_data, std::size_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFEu);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0u && a_data_size_in_bytes <= 255u);

    const std::uint32_t address_mask = (static_cast<std::uint32_t>(a_slave_address)) & I2C_CR2_SADD;
    const std::uint32_t data_size_mask =
        (static_cast<std::uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->a_p_I2C));

    p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START;

    std::size_t bytes           = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            p_registers->TXDR = static_cast<const std::uint8_t*>(a_p_data)[bytes++];
        }

        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_TC))
        {
            bit_flag::set(&(p_registers->CR2), I2C_CR2_STOP);
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_flag>(p_registers);

        bit_flag::set(&(p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);

        if (false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(p_registers->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);
    p_registers->CR2 = 0;

    return { bus_status, bytes };
}

Polling<I2C_master>::Result Polling<I2C_master>::transmit(std::uint8_t a_slave_address,
                                                          const void* a_p_data,
                                                          std::size_t a_data_size_in_bytes,
                                                          std::uint32_t a_timeout)
{
    cml_assert(a_slave_address <= 0xFEu);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0u && a_data_size_in_bytes <= 255u);
    cml_assert(a_timeout > 0u);

    std::uint32_t start = ms_tick_counter::get();

    const std::uint32_t address_mask   = static_cast<std::uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const std::uint32_t data_size_mask = static_cast<std::uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->a_p_I2C));

    p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START;

    std::size_t bytes           = 0u;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= various::tick_diff(ms_tick_counter::get(), start))
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            p_registers->TXDR = static_cast<const std::uint8_t*>(a_p_data)[bytes++];
        }

        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_TC))
        {
            bit_flag::set(&(p_registers->CR2), I2C_CR2_STOP);
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_flag>(p_registers);

        bit_flag::set(&(p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);

        if (false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(p_registers->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);
    p_registers->CR2 = 0;

    return { bus_status, bytes };
}

Polling<I2C_master>::Result
Polling<I2C_master>::receive(std::uint8_t a_slave_address, void* a_p_data, std::size_t a_data_size_in_bytes)
{
    cml_assert(a_slave_address <= 0xFEu);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0u && a_data_size_in_bytes <= 255u);

    const std::uint32_t address_mask   = static_cast<std::uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const std::uint32_t data_size_mask = static_cast<std::uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->a_p_I2C));

    p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_RD_WRN;

    std::size_t bytes           = 0u;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<std::uint8_t*>(a_p_data)[bytes++] = static_cast<std::uint8_t>(p_registers->RXDR);

            if (a_data_size_in_bytes == bytes)
            {
                bit_flag::set(&(p_registers->CR2), I2C_CR2_STOP);
            }
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_flag>(p_registers);

        bit_flag::set(&(p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);

        if (false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(p_registers->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);
    p_registers->CR2 = 0u;

    return { bus_status, bytes };
}

Polling<I2C_master>::Result Polling<I2C_master>::receive(std::uint8_t a_slave_address,
                                                         void* a_p_data,
                                                         std::size_t a_data_size_in_bytes,
                                                         std::uint32_t a_timeout)
{
    cml_assert(a_slave_address <= 0xFEu);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0u && a_data_size_in_bytes <= 255u);
    cml_assert(a_timeout > 0u);

    std::uint32_t start = ms_tick_counter::get();

    const std::uint32_t address_mask   = static_cast<std::uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const std::uint32_t data_size_mask = static_cast<std::uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->a_p_I2C));

    p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_RD_WRN;

    std::size_t bytes           = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF) && false == error &&
           a_timeout >= various::tick_diff(ms_tick_counter::get(), start))
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<std::uint8_t*>(a_p_data)[bytes++] = static_cast<std::uint8_t>(p_registers->RXDR);

            if (a_data_size_in_bytes == bytes)
            {
                bit_flag::set(&(p_registers->CR2), I2C_CR2_STOP);
            }
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_flag>(p_registers);

        bit_flag::set(&(p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);

        if (false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF))
        {
            bit_flag::set(&(p_registers->CR2), I2C_CR2_STOP);
        }
    }

    bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);
    p_registers->CR2 = 0u;

    return { bus_status, bytes };
}

Polling<I2C_slave>::Result Polling<I2C_slave>::transmit(const void* a_p_data, std::size_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255u);

    constexpr std::uint32_t error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->a_p_I2C));

    std::size_t bytes           = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while ((false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF) &&
            false == bit_flag::is(p_registers->ISR, I2C_ISR_NACKF)) &&
           false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(p_registers->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            p_registers->TXDR = static_cast<const std::uint8_t*>(a_p_data)[bytes++];
        }

        error = bit::is_any(p_registers->ISR, error_mask);
    }

    if (true == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF) && true == bit_flag::is(p_registers->ISR, I2C_ISR_NACKF))
    {
        bit_flag::set(&(p_registers->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_flag>(p_registers);

        bit_flag::set(&(p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);
    }

    bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}
Polling<I2C_slave>::Result
Polling<I2C_slave>::transmit(const void* a_p_data, std::size_t a_data_size_in_bytes, std::uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255u);
    cml_assert(a_timeout > 0);

    std::uint32_t start = ms_tick_counter::get();

    constexpr std::uint32_t error_mask = I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR;

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->a_p_I2C));

    std::size_t bytes           = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while ((false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF) &&
            false == bit_flag::is(p_registers->ISR, I2C_ISR_NACKF)) &&
           false == error && a_timeout >= various::tick_diff(ms_tick_counter::get(), start))
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(p_registers->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_TXE) && bytes < a_data_size_in_bytes)
        {
            p_registers->TXDR = static_cast<const std::uint8_t*>(a_p_data)[bytes++];
        }

        error = bit::is_any(p_registers->ISR, error_mask);
    }

    if (true == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF) && true == bit_flag::is(p_registers->ISR, I2C_ISR_NACKF))
    {
        bit_flag::set(&(p_registers->ICR), I2C_ICR_NACKCF);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_flag>(p_registers);

        bit_flag::set(&(p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);
    }

    bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}
Polling<I2C_slave>::Result Polling<I2C_slave>::receive(void* a_p_data, std::size_t a_data_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->a_p_I2C));

    std::size_t bytes           = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, I2C_ISR_STOPF) && false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(p_registers->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_RXNE))
        {
            const std::uint8_t rxdr = static_cast<std::uint8_t>(p_registers->RXDR);

            if (bytes < a_data_size_in_bytes)
            {
                static_cast<std::uint8_t*>(a_p_data)[bytes++] = rxdr;
            }
        }

        error = is_error(p_registers);
    }

    if (false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<std::uint8_t*>(a_p_data)[bytes++] = static_cast<std::uint8_t>(p_registers->RXDR);
        }
    }
    else
    {
        bus_status = get_Bus_status_flag<Result::Bus_flag>(p_registers);

        bit_flag::set(&(p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);
    }

    bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}
Polling<I2C_slave>::Result
Polling<I2C_slave>::receive(void* a_p_data, std::size_t a_data_size_in_bytes, std::uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255u);
    cml_assert(a_timeout > 0u);

    std::uint32_t start = ms_tick_counter::get();

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->a_p_I2C));

    std::size_t bytes           = 0;
    bool error                  = false;
    Result::Bus_flag bus_status = Result::Bus_flag::ok;

    while (false == bit_flag::is(p_registers->ICR, I2C_ICR_STOPCF) && false == error &&
           a_timeout >= various::tick_diff(ms_tick_counter::get(), start))
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_ADDR))
        {
            bit_flag::set(&(p_registers->ICR), I2C_ICR_ADDRCF);
        }

        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_RXNE))
        {
            const std::uint8_t rxdr = static_cast<std::uint8_t>(p_registers->RXDR);

            if (bytes < a_data_size_in_bytes)
            {
                static_cast<std::uint8_t*>(a_p_data)[bytes++] = rxdr;
            }
        }

        error = is_error(p_registers);
    }

    if (false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, I2C_ISR_RXNE) && bytes < a_data_size_in_bytes)
        {
            static_cast<std::uint8_t*>(a_p_data)[bytes++] = static_cast<std::uint8_t>(p_registers->RXDR);
        }
    }
    else
    {
        bus_status = get_Bus_status_flag<Result::Bus_flag>(p_registers);

        bit_flag::set(&(p_registers->ICR),
                      I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                          I2C_ICR_NACKCF);
    }

    bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);

    return { bus_status, bytes };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif