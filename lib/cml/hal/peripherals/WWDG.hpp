#pragma once

/*
 *   Name: WWDG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L452xx
#include <soc/stm32l452xx/peripherals/WWDG.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/WWDG.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L452xx
using WWDG = soc::stm32l452xx::peripherals::WWDG;
#endif // STM32L452xx

#ifdef STM32L011xx
using WWDG = soc::stm32l011xx::peripherals::WWDG;
#endif // STM32L011xx

} // namespace peripherals
} // namespace hal
} // namespace cml