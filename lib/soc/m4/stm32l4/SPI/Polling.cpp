/*
 *   Name: Polling.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/SPI/Polling.hpp>

// soc
#include <soc/m4/stm32l4/SPI/common.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/utils/tick_counter.hpp>
#include <cml/utils/wait_until.hpp>

namespace {

using namespace cml;
using namespace soc::m4::stm32l4;

enum class Direction_flag : uint32_t
{
    transmit,
    receive
};

constexpr Direction_flag operator|(Direction_flag a_f1, Direction_flag a_f2)
{
    return static_cast<Direction_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr Direction_flag operator&(Direction_flag a_f1, Direction_flag a_f2)
{
    return static_cast<Direction_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr Direction_flag operator|=(Direction_flag& a_f1, Direction_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

bool is_error(SPI_TypeDef* a_p_registers, Direction_flag a_direction)
{
    return bit::is_any(a_p_registers->SR,
                       SPI_SR_FRE | (Direction_flag::receive == a_direction ? SPI_SR_OVR : 0) | SPI_SR_MODF |
                           SPI_SR_CRCERR);
}

void clear_overrun(SPI_TypeDef* a_p_registers)
{
    volatile uint32_t t1 = a_p_registers->DR;
    volatile uint32_t t2 = a_p_registers->SR;

    unused(t1);
    unused(t2);
}

} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;
using namespace cml::utils;

Polling<SPI_master>::Result
Polling<SPI_master>::transmit(const void* a_p_data, std::size_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(p_registers->CR1), SPI_CR1_SPE);
    }

    bool error                = false;
    bool busy                 = true;
    std::size_t words         = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    while (true == busy && false == error)
    {
        if (true == bit_flag::is(p_registers->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_error(p_registers, Direction_flag::transmit);
        busy  = bit_flag::is(p_registers->SR, SPI_SR_BSY);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);

    clear_overrun(p_registers);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, words };
}
Polling<SPI_master>::Result Polling<SPI_master>::transmit(const void* a_p_data,
                                                          std::size_t a_data_size_in_words,
                                                          Milliseconds a_timeout,
                                                          GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(p_registers->CR1), SPI_CR1_SPE);
    }

    bool error                = false;
    bool busy                 = true;
    std::size_t words         = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    while (true == busy && false == error && a_timeout >= tick_counter::get() - start)
    {
        if (true == bit_flag::is(p_registers->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_error(p_registers, Direction_flag::transmit);
        busy  = bit_flag::is(p_registers->SR, SPI_SR_BSY);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);

    clear_overrun(p_registers);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, words };
}

Polling<SPI_master>::Result
Polling<SPI_master>::receive(void* a_p_data, std::size_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(p_registers->CR1), SPI_CR1_SPE);
    }

    bool error                = false;
    bool transmit_enable      = true;
    std::size_t words         = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    while (false == error && words < a_data_size_in_words)
    {
        if (true == transmit_enable && true == bit_flag::is(p_registers->SR, SPI_SR_TXE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))) = static_cast<std::uint16_t>(0xFFFFu);
            }
            else
            {
                *(reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))) = static_cast<std::uint8_t>(0xFFu);
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(p_registers->SR, SPI_SR_RXNE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                static_cast<std::uint16_t*>(a_p_data)[words++] = static_cast<std::uint16_t>(p_registers->DR);
            }
            else
            {
                static_cast<std::uint8_t*>(a_p_data)[words++] = static_cast<std::uint8_t>(p_registers->DR);
            }

            transmit_enable = true;
        }

        error = is_error(p_registers, Direction_flag::receive);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);
    wait_until::all_bits(p_registers->SR, SPI_SR_BSY, true);
    wait_until::any_bit(p_registers->SR, SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, words };
}

Polling<SPI_master>::Result Polling<SPI_master>::receive(void* a_p_data,
                                                         std::size_t a_data_size_in_words,
                                                         Milliseconds a_timeout,
                                                         GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(p_registers->CR1), SPI_CR1_SPE);
    }

    bool error                = false;
    bool transmit_enable      = true;
    std::size_t words         = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    while (false == error && words < a_data_size_in_words &&
           a_timeout >= tick_counter::get() - start)
    {
        if (true == transmit_enable && true == bit_flag::is(p_registers->SR, SPI_SR_TXE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))) = static_cast<std::uint16_t>(0xFFFFu);
            }
            else
            {
                *(reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))) = static_cast<std::uint8_t>(0xFFu);
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(p_registers->SR, SPI_SR_RXNE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                static_cast<std::uint16_t*>(a_p_data)[words++] = static_cast<std::uint16_t>(p_registers->DR);
            }
            else
            {
                static_cast<std::uint8_t*>(a_p_data)[words++] = static_cast<std::uint8_t>(p_registers->DR);
            }

            transmit_enable = true;
        }

        error = is_error(p_registers, Direction_flag::receive);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);
    wait_until::all_bits(p_registers->SR, SPI_SR_BSY, true);
    wait_until::any_bit(p_registers->SR, SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, words };
}

Polling<SPI_master>::Result Polling<SPI_master>::transmit_receive(const void* a_p_tx_data,
                                                                  void* a_p_rx_data,
                                                                  std::size_t a_tx_rx_data_size_in_words,
                                                                  GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_tx_data);
    cml_assert(nullptr != a_p_rx_data);
    cml_assert(a_tx_rx_data_size_in_words > 0);

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(p_registers->CR1), SPI_CR1_SPE);
    }

    bool error                = false;
    bool transmit_enable      = true;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    std::size_t tx_idx = 0;
    std::size_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words)
    {
        if (true == transmit_enable && true == bit_flag::is(p_registers->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(p_registers->SR, SPI_SR_RXNE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                std::uint16_t t = static_cast<std::uint16_t>(p_registers->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<std::uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                std::uint8_t t = static_cast<std::uint8_t>(p_registers->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<std::uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_error(p_registers, Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);
    wait_until::all_bits(p_registers->SR, SPI_SR_BSY, true);
    wait_until::any_bit(p_registers->SR, SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, rx_idx };
}

Polling<SPI_master>::Result Polling<SPI_master>::transmit_receive(const void* a_p_tx_data,
                                                                  void* a_p_rx_data,
                                                                  std::size_t a_tx_rx_data_size_in_words,
                                                                  Milliseconds a_timeout,
                                                                  GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_tx_data);
    cml_assert(nullptr != a_p_rx_data);
    cml_assert(a_tx_rx_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(p_registers->CR1), SPI_CR1_SPE);
    }

    bool error                = false;
    bool transmit_enable      = true;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    std::size_t tx_idx = 0;
    std::size_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words &&
           a_timeout >= tick_counter::get() - start)
    {
        if (true == transmit_enable && true == bit_flag::is(p_registers->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(p_registers->SR, SPI_SR_RXNE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                std::uint16_t t = static_cast<std::uint16_t>(p_registers->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<std::uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                std::uint8_t t = static_cast<std::uint8_t>(p_registers->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<std::uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_error(p_registers, Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);
    wait_until::all_bits(p_registers->SR, SPI_SR_BSY, true);
    wait_until::any_bit(p_registers->SR, SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(p_registers->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, rx_idx };
}

Polling<SPI_slave>::Result Polling<SPI_slave>::transmit(const void* a_p_data, std::size_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    bool error                = false;
    bool busy                 = true;
    std::size_t words         = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    while (true == busy && false == error)
    {
        if (true == bit_flag::is(p_registers->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_error(p_registers, Direction_flag::transmit);
        busy  = bit_flag::is(p_registers->SR, SPI_SR_BSY);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);

    clear_overrun(p_registers);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    return { bus_flag, words };
}

Polling<SPI_slave>::Result
Polling<SPI_slave>::transmit(const void* a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    bool error                = false;
    bool busy                 = true;
    std::size_t words         = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    while (true == busy && false == error && a_timeout >= tick_counter::get() - start)
    {
        if (true == bit_flag::is(p_registers->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_error(p_registers, Direction_flag::transmit);
        busy  = bit_flag::is(p_registers->SR, SPI_SR_BSY);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);

    clear_overrun(p_registers);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    return { bus_flag, words };
}

Polling<SPI_slave>::Result Polling<SPI_slave>::receive(void* a_p_data, std::size_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    bool error                = false;
    std::size_t words         = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    while (false == error && words < a_data_size_in_words)
    {
        if (true == bit_flag::is(p_registers->SR, SPI_SR_RXNE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                static_cast<std::uint16_t*>(a_p_data)[words++] = static_cast<std::uint16_t>(p_registers->DR);
            }
            else
            {
                static_cast<std::uint8_t*>(a_p_data)[words++] = static_cast<std::uint8_t>(p_registers->DR);
            }
        }

        error = is_error(p_registers, Direction_flag::receive);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);
    wait_until::all_bits(p_registers->SR, SPI_SR_BSY, true);
    wait_until::any_bit(p_registers->SR, SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    return { bus_flag, words };
}

Polling<SPI_slave>::Result
Polling<SPI_slave>::receive(void* a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    bool error                = false;
    std::size_t words         = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    while (false == error && words < a_data_size_in_words &&
           a_timeout >= tick_counter::get() - start)
    {
        if (true == bit_flag::is(p_registers->SR, SPI_SR_RXNE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                static_cast<std::uint16_t*>(a_p_data)[words++] = static_cast<std::uint16_t>(p_registers->DR);
            }
            else
            {
                static_cast<std::uint8_t*>(a_p_data)[words++] = static_cast<std::uint8_t>(p_registers->DR);
            }
        }

        error = is_error(p_registers, Direction_flag::receive);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);
    wait_until::all_bits(p_registers->SR, SPI_SR_BSY, true);
    wait_until::any_bit(p_registers->SR, SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    return { bus_flag, words };
}

Polling<SPI_slave>::Result
Polling<SPI_slave>::transmit_receive(const void* a_p_tx_data, void* a_p_rx_data, std::size_t a_tx_rx_data_size_in_words)
{
    cml_assert(nullptr != a_p_tx_data);
    cml_assert(nullptr != a_p_rx_data);
    cml_assert(a_tx_rx_data_size_in_words > 0);

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    bool error                = false;
    bool transmit_enable      = true;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    std::size_t tx_idx = 0;
    std::size_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words)
    {
        if (true == transmit_enable && true == bit_flag::is(p_registers->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile uint16_t*>(&(p_registers->DR))) =
                    static_cast<const uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(p_registers->DR))) =
                    static_cast<const uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(p_registers->SR, SPI_SR_RXNE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                std::uint16_t t = static_cast<std::uint16_t>(p_registers->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<std::uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                std::uint8_t t = static_cast<std::uint8_t>(p_registers->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<std::uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_error(p_registers, Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);
    wait_until::all_bits(p_registers->SR, SPI_SR_BSY, true);
    wait_until::any_bit(p_registers->SR, SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    return { bus_flag, rx_idx };
}

Polling<SPI_slave>::Result Polling<SPI_slave>::transmit_receive(const void* a_p_tx_data,
                                                                void* a_p_rx_data,
                                                                std::size_t a_tx_rx_data_size_in_words,
                                                                Milliseconds a_timeout)
{
    cml_assert(nullptr != a_p_tx_data);
    cml_assert(nullptr != a_p_rx_data);
    cml_assert(a_tx_rx_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(this->p_SPI));

    bool error                = false;
    bool transmit_enable      = true;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    std::size_t tx_idx = 0;
    std::size_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words &&
           a_timeout >= tick_counter::get() - start)
    {
        if (true == transmit_enable && true == bit_flag::is(p_registers->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                *(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))) =
                    static_cast<const std::uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(p_registers->SR, SPI_SR_RXNE))
        {
            if (bit_flag::get(p_registers->CR2, SPI_CR2_DS) > (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2))
            {
                std::uint16_t t = static_cast<std::uint16_t>(p_registers->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<std::uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                std::uint8_t t = static_cast<std::uint8_t>(p_registers->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<std::uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_error(p_registers, Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::any_bit(p_registers->SR, SPI_SR_FTLVL, true);
    wait_until::all_bits(p_registers->SR, SPI_SR_BSY, true);
    wait_until::any_bit(p_registers->SR, SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_Bus_flag<Result::Bus_flag>(p_registers);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_errors(p_registers);
        }
    }

    return { bus_flag, rx_idx };
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif