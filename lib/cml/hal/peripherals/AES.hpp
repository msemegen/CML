#pragma once

/*
 *   Name: AES.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/stm32l4/peripherals/AES.hpp>
#endif // STM32L4

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
#if defined(STM32L422xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L462xx)
using AES = soc::stm32l4::peripherals::AES;
#endif
#endif // STM32L4

} // namespace peripherals
} // namespace hal
} // namespace cml