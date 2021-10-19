#pragma once

/*
 *   Name: Basic_timer.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/Basic_timer/Basic_timer.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using Basic_timer = soc::m4::stm32l4::Basic_timer;
#endif
} // namespace hal
} // namespace cml