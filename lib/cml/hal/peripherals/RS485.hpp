#pragma once

/*
 *   Name: RS485.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx
#include <soc/stm32l452xx/peripherals/RS485.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/RS485.hpp>
#endif

namespace cml {
namespace hal {

#ifdef STM32L452xx
using RS485 = soc::stm32l452xx::peripherals::RS485;
#endif // STM32L452xx

#ifdef STM32L011xx
using RS485 = soc::stm32l011xx::peripherals::RS485;
#endif // STM32L011xx

} // namespace hal
} // namespace cml
