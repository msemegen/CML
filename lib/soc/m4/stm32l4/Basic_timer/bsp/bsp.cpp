/*
 *   Name: bsp.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/Basic_timer/bsp/bsp.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>

namespace {

using namespace soc::m4::stm32l4;

Interrupt<Basic_timer>* irq_context[] = { nullptr
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
                                     ,
                                     nullptr
#endif
};

} // namespace

extern "C" {

void TIM6_DAC_IRQHandler()
{
    cml_assert(nullptr != irq_context[0]);
    basic_timer_interrupt_handler(irq_context[0]);
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
void TIM7_IRQHandler()
{
    cml_assert(nullptr != timers[1]);
    basic_timer_interrupt_handler(timers[1]);
}
#endif
}

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;
Interrupt<Basic_timer>::Interrupt(Basic_timer* a_p_timer, IRQn_Type a_irqn)
    : p_timer(a_p_timer)
    , irqn(a_irqn)
{
    cml_assert(nullptr == irq_context[this->p_timer->get_idx()]);
    irq_context[this->p_timer->get_idx()] = this;
}

Interrupt<Basic_timer>::~Interrupt()
{
    cml_assert(nullptr != irq_context[this->get_handle()->get_idx()]);

    this->disable();

    for (std::size_t i = 0; i < std::extent<decltype(irq_context)>::value; i++)
    {
        if (static_cast<Interrupt<Basic_timer>*>(this) == irq_context[i])
        {
            irq_context[i] = nullptr;
            break;
        }
    }
}

template<> void rcc<Basic_timer, 6>::enable(bool a_enable_in_lp)
{
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos);
    }
}
template<> void rcc<Basic_timer, 6>::disable()
{
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos);
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos);
}
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
template<> void rcc<Basic_timer, 7>::enable(bool a_enable_in_lp)
{
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos);
    }
}
template<> void rcc<Basic_timer, 7>::disable()
{
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM7EN_Pos);
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM7SMEN_Pos);
}
#endif

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif