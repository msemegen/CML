/*
 *   Name: Systick.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/Systick/Systick.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {
using namespace soc;

Systick* irq_context[1] = { nullptr };
} // namespace

extern "C" {
void SysTick_Handler()
{
    systick_interrupt_handler();
}
}

namespace soc {
using namespace cml;
using namespace m4;

void systick_interrupt_handler()
{
    cml_assert(nullptr != irq_context[0]);

    if (nullptr != irq_context[0]->interrupt.callback.function)
    {
        irq_context[0]->interrupt.callback.function(irq_context[0]->interrupt.callback.p_user_data);
    }
}

void Systick::enable(std::uint32_t a_start_value, Prescaler a_prescaler)
{
    cml_assert(a_start_value > 0);
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    SysTick->CTRL = 0;
    SysTick->LOAD = a_start_value;
    SysTick->VAL  = 0;
    SysTick->CTRL = static_cast<std::uint32_t>(a_prescaler) | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void Systick::disable()
{
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    SysTick->CTRL = 0;
}

bool Systick::is_enabled()
{
    return bit_flag::is(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

#ifdef M4
void Systick::Interrupt::enable(const IRQ_config& a_irq_config)
{
    cml_assert(nullptr != this->p_systick);
    cml_assert(nullptr == irq_context[0]);

    irq_context[0] = this->p_systick;

    NVIC_SetPriority(
        SysTick_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(SysTick_IRQn);
}
#endif

void Systick::Interrupt::disable()
{
    cml_assert(nullptr != this->p_systick);

    NVIC_DisableIRQ(SysTick_IRQn);

    irq_context[0] = nullptr;
}

void Systick::Interrupt::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != this->p_systick);

    Interrupt_guard guard;
    this->callback = a_callback;
}

void Systick::Interrupt::unregister_callback()
{
    cml_assert(nullptr != this->p_systick);

    Interrupt_guard guard;
    this->callback = { nullptr, nullptr };
}
} // namespace soc
