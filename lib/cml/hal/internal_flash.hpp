#pragma once

/*
 *   Name: internal_flash.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#ifdef STM32L4
#include <soc/m4/stm32l4/internal_flash/internal_flash.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using internal_flash = soc::m4::stm32l4::internal_flash;
#endif
} // namespace hal
} // namespace cml