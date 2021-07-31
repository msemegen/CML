#pragma once

/*
 *   Name: rcc.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/rcc.hpp>
#endif // STM32L4

namespace cml {
namespace hal {

#ifdef STM32L4
template<typename Perihperal_t> using rcc = soc::m4::stm32l4::rcc<Perihperal_t>;
#endif // STM32L4

} // namespace hal
} // namespace cml