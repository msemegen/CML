/*
 *   Name: USART.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/USART/USART.hpp>

// soc
#include <soc/system_timer.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;
using namespace cml::utils;

USART::~USART()
{
    if (true == bit_flag::is(this->p_registers->CR1, USART_CR1_UE))
    {
        this->disable();
    }
}

bool USART::enable(const Enable_config& a_config, const Frame_format& a_frame_format, std::uint32_t a_timeout_ms)
{
    cml_assert(0 != a_config.baud_rate);
    cml_assert(0 != a_config.clock_freq_Hz);

    cml_assert(various::get_enum_incorrect_value<Enable_config::Flow_control_flag>() != a_config.flow_control);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Stop_bits>() != a_config.stop_bits);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Sampling_method>() != a_config.sampling_method);

    cml_assert(various::get_enum_incorrect_value<Frame_format::Parity>() != a_frame_format.parity);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Word_length>() != a_frame_format.word_length);

    cml_assert(a_timeout_ms > 0);

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

    this->p_registers->CR2 = static_cast<std::uint32_t>(a_config.stop_bits);
    this->p_registers->CR3 =
        static_cast<std::uint32_t>(a_config.flow_control) | static_cast<std::uint32_t>(a_config.sampling_method);

    this->p_registers->CR1 = static_cast<std::uint32_t>(a_config.oversampling) |
                             static_cast<std::uint32_t>(a_config.mode) |
                             static_cast<std::uint32_t>(a_frame_format.parity) |
                             static_cast<std::uint32_t>(a_frame_format.word_length) | USART_CR1_UE;

    std::uint32_t wait_flag = (true == bit_flag::is(this->p_registers->CR1, USART_CR1_RE) ? USART_ISR_REACK : 0) |
                              (true == bit_flag::is(this->p_registers->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0);

    return wait_until::all_bits(&(this->p_registers->ISR), wait_flag, false, start, a_timeout_ms);
}

void USART::disable()
{
    this->p_registers->CR1 = 0;
    this->p_registers->CR2 = 0;
    this->p_registers->CR3 = 0;
}

USART::Enable_config USART::get_Enable_config() const
{
    return Enable_config();
}

USART::Frame_format USART::get_Frame_format() const
{
    return Frame_format();
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif