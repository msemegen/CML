/*
    Name: systick.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <soc/systick.hpp>

// cml
#include <cml/bit_flag.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

namespace {

using namespace soc;

static  systick::Tick_callback callback;

} // namespace

extern "C" {

void SysTick_Handler()
{
    if (nullptr != callback.function)
    {
        callback.function(callback.p_user_data);
    }
}

} // extern "C"

namespace soc {

using namespace cml;

void systick::enable(uint32_t a_start_value, Prescaler a_prescaler, uint32_t a_priority)
{
    assert(a_start_value > 0);

    NVIC_SetPriority(SysTick_IRQn, a_priority);

    SysTick->CTRL = 0;
    SysTick->LOAD = a_start_value;
    SysTick->VAL  = 0;
    SysTick->CTRL = static_cast<uint32_t>(a_prescaler) | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void systick::disable()
{
    SysTick->CTRL = 0;
}

void systick::register_tick_callback(const Tick_callback& a_callback)
{
    Interrupt_guard guard;
    callback = a_callback;
}

void systick::unregister_tick_callback()
{
    Interrupt_guard guard;
    callback.function    = nullptr;
    callback.p_user_data = nullptr;
}

bool systick::is_enabled()
{
    return bit_flag::is(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

} // namespace soc
