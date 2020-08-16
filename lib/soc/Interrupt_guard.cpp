/*
    Name: interrupt_guard.cpp

    Copyright(c) 2020 Jay Kickliter
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <soc/interrupt_guard.hpp>

// externals
#ifdef STM32L452xx
#include <stm32l4xx.h>
#endif

#ifdef STM32L011xx
#include <stm32l0xx.h>
#endif

namespace soc {

Interrupt_guard::Interrupt_guard()
    : primask(__get_PRIMASK())
{
    __disable_irq();
}

Interrupt_guard::~Interrupt_guard()
{
    __set_PRIMASK(this->primask);
}

} // namespace soc
