#pragma once

/*
 *   Name: RNG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L452xx
#include <soc/stm32l452xx/peripherals/RNG.hpp>
#endif // STM32L452xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L452xx
using RNG = soc::stm32l452xx::peripherals::RNG;
#endif // STM32L452xx

} // namespace peripherals
} // namespace hal
} // namespace cml