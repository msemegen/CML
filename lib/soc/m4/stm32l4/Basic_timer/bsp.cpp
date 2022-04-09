/*
 *   Name: bsp.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/Basic_timer/bsp.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>

namespace {
using namespace soc::m4::stm32l4;

Basic_timer* irq_context[] = { nullptr
#if defined(STM32L431CC) || defined(STM32L431KB) || defined(STM32L431KC) || defined(STM32L431RB) || \
    defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L432KB) || defined(STM32L432KC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L442KC) || defined(STM32L443CC) || defined(STM32L443RC) || \
    defined(STM32L443VC)
                               ,
                               nullptr
#endif
};

} // namespace

extern "C" {
using namespace soc::m4::stm32l4;

void TIM6_DAC_IRQHandler()
{
    cml_assert(nullptr != irq_context[0]);
    Basic_timer_interrupt_handler(irq_context[0]);
}

#if defined(STM32L431CC) || defined(STM32L431KB) || defined(STM32L431KC) || defined(STM32L431RB) || \
    defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L432KB) || defined(STM32L432KC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L442KC) || defined(STM32L443CC) || defined(STM32L443RC) || \
    defined(STM32L443VC)
void TIM7_IRQHandler()
{
    cml_assert(nullptr != irq_context[1]);
    Basic_timer_interrupt_handler(irq_context[1]);
}
#endif
}

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void Basic_timer::Interrupt::set_irq_context()
{
    cml_assert(nullptr == irq_context[this->p_timer->idx]);
    irq_context[this->p_timer->idx] = this->p_timer;
}

void Basic_timer::Interrupt::clear_irq_context()
{
    cml_assert(nullptr != irq_context[this->p_timer->idx]);

    irq_context[this->p_timer->idx] = nullptr;
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
#if defined(STM32L431CC) || defined(STM32L431KB) || defined(STM32L431KC) || defined(STM32L431RB) || \
    defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L432KB) || defined(STM32L432KC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L442KC) || defined(STM32L443CC) || defined(STM32L443RC) || \
    defined(STM32L443VC)
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