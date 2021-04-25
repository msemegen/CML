#pragma once

/*
 *   Name: internal_flash.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L452xx
#include <soc/stm32l452xx/internal_flash.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/internal_flash.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {

#ifdef STM32L452xx
using internal_flash = soc::stm32l452xx::internal_flash;
#endif // STM32L452xx

#ifdef STM32L011xx
using internal_flash = soc::stm32l011xx::internal_flash;
#endif // STM32L011xx

} // namespace hal
} // namespace cml