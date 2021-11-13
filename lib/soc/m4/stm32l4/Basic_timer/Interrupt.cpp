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


namespace soc {
namespace m4 {
namespace stm32l4 {

void basic_timer_interrupt_handler(Interrupt<Basic_timer>* a_p_this)
{
    a_p_this->callback.function(a_p_this->get_handle(), a_p_this->callback.p_user_data);

    static_cast<TIM_TypeDef*>(*(a_p_this->get_handle()))->SR = 0;
}

void Interrupt<Basic_timer>::enable(const IRQ_config& a_irq_config)
{
    this->set_irq_context();

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);
}

void Interrupt<Basic_timer>::disable()
{
    NVIC_DisableIRQ(this->irqn);

    this->clear_irq_context();
}

void Interrupt<Basic_timer>::register_callback(const Callback& a_callback)
{
    Interrupt_guard guard;

    static_cast<TIM_TypeDef*>(*(this->p_timer))->DIER = TIM_DIER_UIE;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif