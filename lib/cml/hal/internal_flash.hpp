#pragma once

/*
 *   Name: internal_flash.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L4
#include <soc/stm32l4/internal_flash.hpp>
#endif // STM32L4

#ifdef STM32L011xx
#include <soc/stm32l011xx/internal_flash.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {

#ifdef STM32L4
using internal_flash = soc::stm32l4::internal_flash;
#endif // STM32L4

#ifdef STM32L011xx
using internal_flash = soc::stm32l011xx::internal_flash;
#endif // STM32L011xx

} // namespace hal
} // namespace cml