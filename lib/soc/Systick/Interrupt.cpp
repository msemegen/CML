/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/Systick/Interrupt.hpp>

namespace {

using namespace soc;
using namespace soc::m4::stm32l4;

Interrupt<Systick>* irq_context[1] = { nullptr };

} // namespace

extern "C" {

void SysTick_Handler()
{
    systick_interrupt_handler();
}
}

namespace soc {
using namespace m4;

void systick_interrupt_handler()
{
    cml_assert(nullptr != irq_context[0]);

    if (nullptr != irq_context[0]->callback.function)
    {
        irq_context[0]->callback.function(irq_context[0]->callback.p_user_data);
    }
}

#ifdef M4
void Interrupt<Systick>::enable(const IRQ_config& a_irq_config)
{
    cml_assert(nullptr != this->p_systick);
    cml_assert(nullptr == irq_context[0]);

    irq_context[0] = this;

    NVIC_SetPriority(
        SysTick_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(SysTick_IRQn);
}
#endif

void Interrupt<Systick>::disable()
{
    cml_assert(nullptr != this->p_systick);

    NVIC_DisableIRQ(SysTick_IRQn);

    irq_context[0] = nullptr;
}

void Interrupt<Systick>::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != this->p_systick);

    Interrupt_guard guard;
    this->callback = a_callback;
}

void Interrupt<Systick>::unregister_callback()
{
    cml_assert(nullptr != this->p_systick);

    Interrupt_guard guard;
    this->callback = { nullptr, nullptr };
}
} // namespace soc