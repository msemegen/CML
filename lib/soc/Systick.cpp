/*
 *   Name: Systick.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/Systick.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

namespace {

using namespace soc;

Systick* p_systick = nullptr;

} // namespace

extern "C" {

void SysTick_Handler()
{
    systick_interrupt_handler(p_systick);
}

} // extern "C"

namespace soc {

using namespace cml;

void systick_interrupt_handler(Systick* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    if (nullptr != a_p_this->tick_callback.function)
    {
        a_p_this->tick_callback.function(a_p_this->tick_callback.p_user_data);
    }
}

void Systick::enable(uint32_t a_start_value, Prescaler a_prescaler, uint32_t a_priority)
{
    cml_assert(a_start_value > 0);

    NVIC_SetPriority(SysTick_IRQn, a_priority);

    SysTick->CTRL = 0;
    SysTick->LOAD = a_start_value;
    SysTick->VAL  = 0;
    SysTick->CTRL = static_cast<uint32_t>(a_prescaler) | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    p_systick = this;
}

void Systick::disable()
{
    SysTick->CTRL = 0;
    p_systick     = nullptr;
}

void Systick::register_tick_callback(const Tick_callback& a_callback)
{
    Interrupt_guard guard;
    this->tick_callback = a_callback;
}

void Systick::unregister_tick_callback()
{
    Interrupt_guard guard;
    this->tick_callback.function    = nullptr;
    this->tick_callback.p_user_data = nullptr;
}

bool Systick::is_enabled()
{
    return bit_flag::is(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

} // namespace soc
