#pragma once

/*
 *   Name: pwr.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L4
#include <soc/m4/stm32l4/pwr.hpp>
#endif // STM32L4

namespace cml {
namespace hal {

#ifdef STM32L4
using pwr = soc::m4::stm32l4::pwr;
#endif // STM32L4

} // namespace hal
} // namespace cml
