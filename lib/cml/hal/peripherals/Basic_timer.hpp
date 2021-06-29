#pragma once

/*
 *   Name: Basic_timer.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/peripherals/Basic_timer.hpp>
#endif // STM32L4

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
using Basic_timer = soc::m4::stm32l4::peripherals::Basic_timer;
#endif // STM32L4

} // namespace peripherals
} // namespace hal
} // namespace cml