/*
    Name: systick.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <soc/systick.hpp>

//externals

#ifdef STM32L452xx
#include <stm32l4xx.h>
#endif

#ifdef STM32L011xx
#include <stm32l0xx.h>
#endif

//cml
#include <cml/bit.hpp>

//soc
#include <soc/interrupt_guard.hpp>

namespace
{

using namespace soc;

systick::Tick_callback callback;

} // namespace ::

extern "C"
{

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

void systick::enable(uint32_t a_start_value, uint32_t a_priority)
{
    assert(a_start_value > 0);

    NVIC_SetPriority(SysTick_IRQn, a_priority);

    SysTick->CTRL = 0;
    SysTick->LOAD = a_start_value;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
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
    return is_flag(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

} // namespace soc
