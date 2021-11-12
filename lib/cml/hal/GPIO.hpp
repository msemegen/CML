#pragma once

/*
 *   Name: GPIO.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/GPIO/bsp/bsp.hpp>
#include <soc/m4/stm32l4/GPIO/Interrupt.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using GPIO = soc::m4::stm32l4::GPIO;
#endif
} // namespace hal
} // namespace cml