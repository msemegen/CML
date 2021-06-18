#pragma once

/*
 *   Name: ADC.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/stm32l4/rcc.hpp>
#endif // STM32L4

namespace cml {
namespace hal {

#ifdef STM32L4
template<typename T> using rcc = soc::stm32l4::rcc<T>;
#endif // STM32L4

} // namespace hal
} // namespace cml