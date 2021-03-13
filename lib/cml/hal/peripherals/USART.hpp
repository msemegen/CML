#pragma once

/*
 *   Name: USART.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L452xx
#include <soc/stm32l452xx/peripherals/USART.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/USART.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L452xx
using USART = soc::stm32l452xx::peripherals::USART;
#endif // STM32L452xx

#ifdef STM32L011xx
using USART = soc::stm32l011xx::peripherals::USART;
#endif // STM32L011xx

} // namespace peripherals
} // namespace hal
} // namespace cml