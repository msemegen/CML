#pragma once

/*
 *   Name: IRQ.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#ifdef STM32L4
#include <soc/m4/stm32l4/IRQ.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using IRQ = soc::m4::stm32l4::IRQ;
#endif
} // namespace hal
} // namespace cml