/*
 *   Name: RS485.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/RS485/RS485.hpp>

// soc
#include <soc/system_timer.hpp>

// cml
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;
using namespace cml::utils;

bool RS485::enable(const Enable_config& a_config, std::uint32_t a_timeout)
{
    cml_assert(0 != a_config.baud_rate);
    cml_assert(0 != a_config.clock_freq_Hz);

    cml_assert(various::get_enum_incorrect_value<Enable_config::Stop_bits>() != a_config.stop_bits);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Oversampling>() != a_config.oversampling);

    cml_assert(a_timeout > 0);

    std::uint32_t start = system_timer::get();

    switch (a_config.oversampling)
    {
        case Enable_config::Oversampling::_16: {
            this->p_registers->BRR = a_config.clock_freq_Hz / a_config.baud_rate;
        }
        break;

        case Enable_config::Oversampling::_8: {
            const std::uint32_t usartdiv = 2 * a_config.clock_freq_Hz / a_config.baud_rate;
            this->p_registers->BRR       = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;
    }

    this->p_registers->CR3 = USART_CR3_ONEBIT;
    this->p_registers->CR2 =
        static_cast<std::uint32_t>(a_config.stop_bits) | (a_config.address << USART_CR2_ADD_Pos) | USART_CR2_ADDM7;

    this->p_registers->CR1 = static_cast<std::uint32_t>(a_config.oversampling) | USART_CR1_M0 | USART_CR1_UE |
                             USART_CR1_TE | USART_CR1_RE | USART_CR1_MME | USART_CR1_WAKE;

    this->p_registers->RQR = USART_RQR_MMRQ;

    return wait_until::all_bits(
        &(this->p_registers->ISR), USART_ISR_TEACK | USART_ISR_REACK | USART_ISR_RWU, false, start, a_timeout);
}

void RS485::disable()
{
    this->p_registers->CR1 = 0;
    this->p_registers->CR2 = 0;
    this->p_registers->CR3 = 0;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif