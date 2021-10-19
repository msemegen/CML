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

template<> void rcc<USART>::enable<rcc<USART>::Clock_source::HSI, USART1_BASE>(Handle<USART1_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART1SEL_0 | RCC_CCIPR_USART1SEL_1, RCC_CCIPR_USART1SEL_1);
    bit::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN_Pos);
    }
}

template<> void rcc<USART>::enable<rcc<USART>::Clock_source::PCLK1, USART1_BASE>(Handle<USART1_BASE>, bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART1SEL);
    bit::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN_Pos);
    }
}

template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::SYSCLK, USART1_BASE>(Handle<USART1_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART1SEL_0 | RCC_CCIPR_USART1SEL_1, RCC_CCIPR_USART1SEL_0);
    bit::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN_Pos);
    }
}

template<> void rcc<USART>::enable<rcc<USART>::Clock_source::HSI, USART2_BASE>(Handle<USART2_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART2SEL_0 | RCC_CCIPR_USART2SEL_1, RCC_CCIPR_USART2SEL_1);
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART2SMEN_Pos);
    }
}

template<> void rcc<USART>::enable<rcc<USART>::Clock_source::PCLK1, USART2_BASE>(Handle<USART2_BASE>, bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART2SEL_0 | RCC_CCIPR_USART2SEL_1);
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART2SMEN_Pos);
    }
}

template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::SYSCLK, USART2_BASE>(Handle<USART2_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART2SEL_0 | RCC_CCIPR_USART2SEL_1, RCC_CCIPR_USART2SEL_0);
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART2SMEN_Pos);
    }
}

template<> void rcc<USART>::disable<USART1_BASE>(Handle<USART1_BASE>)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART1SEL);
    bit::clear(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN_Pos);
    bit::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN_Pos);
}

template<> void rcc<USART>::disable<USART2_BASE>(Handle<USART2_BASE>)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART2SEL);
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN_Pos);
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART2SMEN_Pos);
}

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<USART>::enable<rcc<USART>::Clock_source::HSI, USART3_BASE>(Handle<USART3_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART3SEL_0 | RCC_CCIPR_USART3SEL_1, RCC_CCIPR_USART3SEL_1);
    bit::set(&(RCC->APB2ENR), RCC_APB1ENR1_USART3EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART3SMEN_Pos);
    }
}

template<> void rcc<USART>::enable<rcc<USART>::Clock_source::PCLK1, USART3_BASE>(Handle<USART3_BASE>, bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART3SEL_0 | RCC_CCIPR_USART3SEL_1);
    bit::set(&(RCC->APB2ENR), RCC_APB1ENR1_USART3EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART3SMEN_Pos);
    }
}

template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::SYSCLK, USART3_BASE>(Handle<USART3_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART3SEL_0 | RCC_CCIPR_USART3SEL_1, RCC_CCIPR_USART3SEL_0);
    bit::set(&(RCC->APB2ENR), RCC_APB1ENR1_USART3EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART3SMEN_Pos);
    }
}

template<> void rcc<USART>::disable<USART3_BASE>(Handle<USART3_BASE>)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART3SEL);
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN_Pos);
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART3SMEN_Pos);
}
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif