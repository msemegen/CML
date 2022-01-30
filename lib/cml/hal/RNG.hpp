#pragma once

/*
 *   Name: RNG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/RNG/Interrupt.hpp>
#include <soc/m4/stm32l4/RNG/Polling.hpp>
#include <soc/m4/stm32l4/RNG/RNG.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using RNG = soc::m4::stm32l4::RNG;
#endif
} // namespace hal
} // namespace cml