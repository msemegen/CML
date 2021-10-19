#pragma once

/*
 *   Name: mcu.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#ifdef STM32L4
#include <soc/m4/stm32l4/mcu/mcu.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using mcu = soc::m4::stm32l4::mcu;
#endif
} // namespace hal
} // namespace cml