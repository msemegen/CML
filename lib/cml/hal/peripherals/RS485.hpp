#pragma once

/*
 *   Name: RS485.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/stm32l4/peripherals/RS485.hpp>
#endif // STM32L4

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/RS485.hpp>
#endif

namespace cml {
namespace hal {

#ifdef STM32L4
using RS485 = soc::stm32l4::peripherals::RS485;
#endif // STM32L4

#ifdef STM32L011xx
using RS485 = soc::stm32l011xx::peripherals::RS485;
#endif // STM32L011xx

} // namespace hal
} // namespace cml
