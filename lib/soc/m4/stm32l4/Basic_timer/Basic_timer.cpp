/*
 *   Name: Basic_timer.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/Basic_timer/Basic_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void Basic_timer::enable(const Enable_config& a_config)
{
    cml_assert(various::get_enum_incorrect_value<Enable_config::Mode>() != a_config.mode);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Autoreload_preload>() != a_config.auto_reload_preload);

    this->p_registers->CNT = 0u;
    this->p_registers->PSC = a_config.prescaler;
    this->p_registers->ARR = a_config.auto_reload;

    this->p_registers->CR1 = TIM_CR1_UIFREMAP | TIM_CR1_ARPE;
    this->p_registers->SR  = 0u;
}

void Basic_timer::disable()
{
    this->p_registers->CR1 = 0x0u;
}

void Basic_timer::start()
{
    bit_flag::set(&(this->p_registers->CR1), TIM_CR1_CEN);
}

void Basic_timer::stop()
{
    bit_flag::clear(&(this->p_registers->CR1), TIM_CR1_CEN);
}

template<> void rcc<Basic_timer>::enable<TIM6_BASE>(Handle<TIM6_BASE>, bool a_enable_in_lp)
{
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos);
    }
}

template<> void rcc<Basic_timer>::disable<TIM6_BASE>(Handle<TIM6_BASE>)
{
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos);
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos);
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
template<> void rcc<Basic_timer>::enable<TIM7_BASE>(Handle<TIM7_BASE>, bool a_enable_in_lp)
{
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos);
    }
}

template<> void rcc<Basic_timer>::disable<TIM7_BASE>(Handle<TIM7_BASE>)
{
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM7EN_Pos);
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM7SMEN_Pos);
}
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif