#pragma once

/*
 *   Name: IWDG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L4
#include <soc/stm32l4/peripherals/IWDG.hpp>
#endif // STM32L4

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/IWDG.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
using IWDG = soc::stm32l4::peripherals::IWDG;
#endif // STM32L4

#ifdef STM32L011xx
using IWDG = soc::stm32l011xx::peripherals::IWDG;
#endif // STM32L011xx

} // namespace peripherals
} // namespace hal
} // namespace cml