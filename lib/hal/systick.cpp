/*
    Name: systick.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <hal/systick.hpp>

//externals

#ifdef STM32L452xx
#include <stm32l4xx.h>
#endif

#ifdef STM32L011xx
#include <stm32l0xx.h>
#endif

//cml
#include <common/bit.hpp>

namespace
{

using namespace cml::common;

uint32 cnt = 0;

void systick_handle_interrupt()
{
    cnt++;
}

} // namespace ::

extern "C"
{
using namespace cml::hal;

void SysTick_Handler()
{
    systick_handle_interrupt();
}

} // extern "C"

namespace cml {
namespace hal {

using namespace cml::common;

void systick::enable(uint32 a_priority)
{
    assert(SystemCoreClock / 1000 > 1);

    NVIC_SetPriority(SysTick_IRQn, a_priority);

    SysTick->CTRL = 0;
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void systick::disable()
{
    SysTick->CTRL = 0;
}

void systick::reset_counter()
{
    cnt = 0;
}

bool systick::is_enabled()
{
    return is_flag(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

common::time_tick systick::get_counter()
{
    return cnt;
}

} // namespace hal
} // namespace cml