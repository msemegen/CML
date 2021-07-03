#pragma once

/*
 *   Name: EXTI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/EXTI.hpp>
#endif

namespace cml {
namespace hal {

#ifdef STM32L4
template<typename Perihperal_t> using EXTI = soc::m4::stm32l4::EXTI<Perihperal_t>;
#endif

} // namespace hal
} // namespace cml