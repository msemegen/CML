/*
 *   Name: Polling.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/RS485/Polling.hpp>
#include <soc/m4/stm32l4/USART/Polling.hpp>

// soc
#include <soc/m4/stm32l4/USART/common.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

Polling<USART>::Result Polling<USART>::transmit(const void* a_p_data, std::size_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_usart));

    bit_flag::set(&(p_registers->ICR), USART_ICR_TCCF);

    std::size_t words                  = 0;
    bool error                         = false;
    Result::Bus_status_flag bus_status = Result::Bus_status_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_TC) && false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_TXE) && words < a_data_size_in_words)
        {
            if (false == bit::is(p_registers->CR1, USART_CR1_PCE_Pos) &&
                (true == bit::is(p_registers->CR1, USART_CR1_M0_Pos) &&
                 false == bit::is(p_registers->CR1, USART_CR1_M1_Pos)))
            {
                p_registers->TDR = (static_cast<const std::uint16_t*>(a_p_data)[words++]) & 0x1FFu;
            }
            else
            {
                p_registers->TDR = (static_cast<const std::uint8_t*>(a_p_data)[words++]) & 0xFFu;
            }
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Polling<USART>::Result::Bus_status_flag>(p_registers);
        bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
    }

    return { bus_status, words };
}

Polling<USART>::Result
Polling<USART>::transmit(const void* a_p_data, std::size_t a_data_size_in_words, std::uint32_t a_timeout_ms)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout_ms > 0);

    std::uint32_t start = system_timer::get();

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_usart));

    bit_flag::set(&(p_registers->ICR), USART_ICR_TCCF);

    std::size_t words                  = 0;
    bool error                         = false;
    Result::Bus_status_flag bus_status = Result::Bus_status_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_TC) && false == error &&
           a_timeout_ms < various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_TXE) && words < a_data_size_in_words)
        {
            if (false == bit::is(p_registers->CR1, USART_CR1_PCE_Pos) &&
                (true == bit::is(p_registers->CR1, USART_CR1_M0_Pos) &&
                 false == bit::is(p_registers->CR1, USART_CR1_M1_Pos)))
            {
                p_registers->TDR = (static_cast<const std::uint16_t*>(a_p_data)[words++]) & 0x1FFu;
            }
            else
            {
                p_registers->TDR = (static_cast<const std::uint8_t*>(a_p_data)[words++]) & 0xFFu;
            }
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Polling<USART>::Result::Bus_status_flag>(p_registers);
        bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
    }

    return { bus_status, words };
}

Polling<USART>::Result Polling<USART>::receive(void* a_p_data, std::size_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_usart));

    bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words                  = 0;
    bool error                         = false;
    Result::Bus_status_flag bus_status = Result::Bus_status_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_IDLE) && false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_RXNE))
        {
            if (words < a_data_size_in_words)
            {
                if (false == bit::is(p_registers->CR1, USART_CR1_PCE_Pos) &&
                    (true == bit::is(p_registers->CR1, USART_CR1_M0_Pos) &&
                     false == bit::is(p_registers->CR1, USART_CR1_M1_Pos)))
                {
                    static_cast<std::uint16_t*>(a_p_data)[words++] = (p_registers->RDR & 0x1FFu);
                }
                else
                {
                    static_cast<std::uint8_t*>(a_p_data)[words++] = (p_registers->RDR & 0xFFu);
                }
            }
            else
            {
                bit_flag::set(&(p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Polling<USART>::Result::Bus_status_flag>(p_registers);
        bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
    }

    return { bus_status, words };
}

Polling<USART>::Result
Polling<USART>::receive(void* a_p_data, std::size_t a_data_size_in_words, std::uint32_t a_timeout_ms)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout_ms > 0);

    std::uint32_t start = system_timer::get();

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_usart));

    bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words                  = 0;
    bool error                         = false;
    Result::Bus_status_flag bus_status = Result::Bus_status_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_IDLE) && false == error &&
           a_timeout_ms >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_RXNE))
        {
            if (words < a_data_size_in_words)
            {
                if (false == bit::is(p_registers->CR1, USART_CR1_PCE_Pos) &&
                    (true == bit::is(p_registers->CR1, USART_CR1_M0_Pos) &&
                     false == bit::is(p_registers->CR1, USART_CR1_M1_Pos)))
                {
                    static_cast<std::uint16_t*>(a_p_data)[words++] = (p_registers->RDR & 0x1FFu);
                }
                else
                {
                    static_cast<std::uint8_t*>(a_p_data)[words++] = (p_registers->RDR & 0xFFu);
                }
            }
            else
            {
                bit_flag::set(&(p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Polling<USART>::Result::Bus_status_flag>(p_registers);
        bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
    }

    return { bus_status, words };
}

Polling<RS485>::Result
Polling<RS485>::transmit(std::uint8_t a_address, const void* a_p_data, std::size_t a_data_size_in_words)
{
    cml_assert(a_address <= 0x7F);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_RS485));

    bit_flag::set(&(p_registers->ICR), USART_ICR_TCCF);

    std::size_t words                  = 0;
    bool error                         = false;
    Result::Bus_status_flag bus_status = Result::Bus_status_flag::ok;

    this->p_flow_control_pin->set_level(GPIO::Level::high);

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_TC) && false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_TXE))
        {
            if (words == 0)
            {
                p_registers->TDR = (static_cast<uint16_t>(a_address) | static_cast<uint16_t>(0x100u));
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                p_registers->TDR = (static_cast<const uint8_t*>(a_p_data)[words - 1]);
                words++;
            }
        }

        error = is_error(p_registers);
    }

    this->p_flow_control_pin->set_level(GPIO::Level::low);

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_status_flag>(p_registers);
        bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
    }

    return { bus_status, words };
}

Polling<RS485>::Result Polling<RS485>::transmit(std::uint8_t a_address,
                                                const void* a_p_data,
                                                std::size_t a_data_size_in_words,
                                                std::uint32_t a_timeout_ms)
{
    cml_assert(a_address <= 0x7F);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout_ms > 0);

    std::uint32_t start = system_timer::get();

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_RS485));

    bit_flag::set(&(p_registers->ICR), USART_ICR_TCCF);

    std::size_t words                  = 0;
    bool error                         = false;
    Result::Bus_status_flag bus_status = Result::Bus_status_flag::ok;

    this->p_flow_control_pin->set_level(GPIO::Level::high);

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_TC) && false == error &&
           a_timeout_ms < various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_TXE))
        {
            if (words == 0)
            {
                p_registers->TDR = (static_cast<uint16_t>(a_address) | static_cast<uint16_t>(0x100u));
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                p_registers->TDR = (static_cast<const uint8_t*>(a_p_data)[words - 1]);
                words++;
            }
        }

        error = is_error(p_registers);
    }

    this->p_flow_control_pin->set_level(GPIO::Level::low);

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_status_flag>(p_registers);
        bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
    }

    return { bus_status, words };
}

Polling<RS485>::Result Polling<RS485>::receive(void* a_p_data, std::size_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_RS485));

    bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words                  = 0;
    bool error                         = false;
    Result::Bus_status_flag bus_status = Result::Bus_status_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_IDLE) && false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_RXNE))
        {
            if (0 == words)
            {
                bit_flag::set(&(p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                static_cast<uint8_t*>(a_p_data)[words - 1] = (p_registers->RDR & 0xFFu);
                words++;
            }
            else
            {
                bit_flag::set(&(p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_error(p_registers);
    }

    bit_flag::set(&(p_registers->ICR), USART_ICR_CMCF);

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_status_flag>(p_registers);
        bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
    }

    return { bus_status, words };
}

Polling<RS485>::Result
Polling<RS485>::receive(void* a_p_data, std::size_t a_data_size_in_words, std::uint32_t a_timeout_ms)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout_ms > 0);

    std::uint32_t start = system_timer::get();

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_RS485));

    bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words                  = 0;
    bool error                         = false;
    Result::Bus_status_flag bus_status = Result::Bus_status_flag::ok;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_IDLE) && false == error &&
           a_timeout_ms >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_RXNE))
        {
            if (0 == words)
            {
                bit_flag::set(&(p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                static_cast<uint8_t*>(a_p_data)[words - 1] = (p_registers->RDR & 0xFFu);
                words++;
            }
            else
            {
                bit_flag::set(&(p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_error(p_registers);
    }

    bit_flag::set(&(p_registers->ICR), USART_ICR_CMCF);

    if (true == error)
    {
        bus_status = get_Bus_status_flag<Result::Bus_status_flag>(p_registers);
        bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
    }

    return { bus_status, words };
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif