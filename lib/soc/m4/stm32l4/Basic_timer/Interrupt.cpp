/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/Basic_timer/Interrupt.hpp>

// cml
#include <cml/debug/assertion.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

namespace {

using namespace soc::m4::stm32l4;

Interrupt<Basic_timer>* timers[] = { nullptr
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
                                     ,
                                     nullptr
#endif
};

} // namespace

extern "C" {

void TIM6_DAC_IRQHandler()
{
    cml_assert(nullptr != timers[0]);
    basic_timer_interrupt_handler(timers[0]);
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

void basic_timer_interrupt_handler(Interrupt<Basic_timer>* a_p_this)
{
    a_p_this->callback.function(a_p_this->get_handle(), a_p_this->callback.p_user_data);

    static_cast<TIM_TypeDef*>(*(a_p_this->get_handle()))->SR = 0;
}

void Interrupt<Basic_timer>::enable(const IRQ& a_irq)
{
    cml_assert(true == a_irq.active);

    NVIC_SetPriority(this->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq.preempt_priority, a_irq.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    timers[this->p_timer->get_id()] = this;
}

void Interrupt<Basic_timer>::disable()
{
    NVIC_DisableIRQ(this->irqn);
    timers[this->p_timer->get_id()] = nullptr;
}

void Interrupt<Basic_timer>::register_callback(const Overload_callback& a_callback)
{
    Interrupt_guard guard;

    static_cast<TIM_TypeDef*>(*(this->p_timer))->DIER = TIM_DIER_UIE;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif