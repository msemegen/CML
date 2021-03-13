#pragma once

/*
 *   Name: CRC32.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx
#include <soc/stm32l452xx/peripherals/crc32.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/peripherals/CRC32.hpp>
#endif // STM32L011xx

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L452xx
using CRC32 = soc::stm32l452xx::peripherals::CRC32;
#endif // STM32L452xx

#ifdef STM32L011xx
using CRC32 = soc::stm32l011xx::peripherals::CRC32;
#endif // STM32L011xx

} // namespace peripherals
} // namespace hal
} // namespace cml