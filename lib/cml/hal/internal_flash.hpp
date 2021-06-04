#pragma once

/*
 *   Name: internal_flash.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#ifdef STM32L4
#include <soc/stm32l4/internal_flash.hpp>
#endif // STM32L4

namespace cml {
namespace hal {

#ifdef STM32L4
using internal_flash = soc::stm32l4::internal_flash;
#endif // STM32L4

} // namespace hal
} // namespace cml