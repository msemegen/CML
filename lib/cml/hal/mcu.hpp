#pragma once

/*
 *   Name: mcu.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L4
#include <soc/stm32l4/mcu.hpp>
#endif // STM32L4

#ifdef STM32L011xx
#include <soc/stm32l011xx/mcu.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {

#ifdef STM32L4
using mcu = soc::stm32l4::mcu;
#endif // STM32L4

#ifdef STM32L011xx
using mcu = soc::stm32l011xx::mcu;
#endif // STM32L011xx

} // namespace hal
} // namespace cml
