#pragma once

/*
 *   Name: GPIO.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/stm32l4/peripherals/GPIO.hpp>
#endif // STM32L4

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
using GPIO                     = soc::stm32l4::peripherals::GPIO;
template<typename T> using rcc = soc::stm32l4::rcc<T>;
#endif // STM32L4

} // namespace peripherals
} // namespace hal
} // namespace cml