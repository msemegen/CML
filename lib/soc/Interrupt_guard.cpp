/*
 *   Name: Interrupt_guard.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/interrupt_guard.hpp>

// externals
#ifdef STM32L4
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
