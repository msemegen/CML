#pragma once

/*
 *   Name: USART.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/USART/bsp.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
using USART = soc::m4::stm32l4::USART;
#endif
} // namespace hal
} // namespace cml