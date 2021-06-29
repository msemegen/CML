#pragma once

/*
 *   Name: RS485.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/peripherals/RS485.hpp>
#endif // STM32L4

namespace cml {
namespace hal {

#ifdef STM32L4
using RS485 = soc::m4::stm32l4::peripherals::RS485;
#endif // STM32L4

} // namespace hal
} // namespace cml
