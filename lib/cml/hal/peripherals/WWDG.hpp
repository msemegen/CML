#pragma once

/*
 *   Name: WWDG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/stm32l4/peripherals/WWDG.hpp>
#endif // STM32L4

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
using WWDG = soc::stm32l4::peripherals::WWDG;
#endif // STM32L452xx

} // namespace peripherals
} // namespace hal
} // namespace cml