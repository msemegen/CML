#pragma once

/*
 *   Name: common.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

//std
#include <cstdint>

// externals
#include <stm32l4xx.h>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
inline bool is_error(USART_TypeDef* a_p_registers)
{
    return cml::bit::is_any(a_p_registers->ISR, USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
}

template<typename Status_flag_t> Status_flag_t get_Bus_status_flag(USART_TypeDef* a_p_registers)
{
    Status_flag_t ret       = Status_flag_t::ok;
    const std::uint32_t isr = a_p_registers->ISR;

    if (true == cml::bit_flag::is(isr, USART_ISR_PE))
    {
        ret |= Status_flag_t::parity_error;
    }
    if (true == cml::bit_flag::is(isr, USART_ISR_FE))
    {
        ret |= Status_flag_t::framing_error;
    }
    if (true == cml::bit_flag::is(isr, USART_ISR_ORE))
    {
        ret |= Status_flag_t::overrun;
    }
    if (true == cml::bit_flag::is(isr, USART_ISR_NE))
    {
        ret |= Status_flag_t::noise_detected;
    }

    return ret;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc