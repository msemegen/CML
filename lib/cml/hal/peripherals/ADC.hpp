#pragma once

/*
 *   Name: ADC.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L4
#include <soc/stm32l4/peripherals/ADC.hpp>
#endif // STM32L4

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/ADC.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
using ADC = soc::stm32l452xx::peripherals::ADC;
#endif // STM32L4

#ifdef STM32L011xx
using ADC = soc::stm32l011xx::peripherals::ADC;
#endif // STM32L011xx

} // namespace peripherals
} // namespace hal
} // namespace cml