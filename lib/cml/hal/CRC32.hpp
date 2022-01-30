#pragma once

/*
 *   Name: CRC32.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/CRC32/CRC32.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using CRC32 = soc::m4::stm32l4::CRC32;
#endif
} // namespace hal
} // namespace cml