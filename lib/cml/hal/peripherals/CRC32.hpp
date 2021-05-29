#pragma once

/*
 *   Name: CRC32.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/stm32l4/peripherals/crc32.hpp>
#endif // STM32L4

namespace cml {
namespace hal {
namespace peripherals {

#ifdef STM32L4
using CRC32 = soc::stm32l4::peripherals::CRC32;
#endif // STM32L4

} // namespace peripherals
} // namespace hal
} // namespace cml