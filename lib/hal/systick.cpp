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

//mhl
#include <common/bit.hpp>

extern "C"
{
using namespace cml::hal;

void SysTick_Handler()
{
    systick_handle_interrupt(&(c_systick::get_instance()));
}

} // extern "C"

namespace cml {
namespace hal {

using namespace common;

void c_systick::enable()
{
    SysTick->CTRL = 0;
    SysTick->LOAD = SystemCoreClock / 1000;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void c_systick::disable()
{
    SysTick->CTRL = 0;
}

bool c_systick::is_enabled() const
{
    return is_flag(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

} // namespace hal
} // namespace cml