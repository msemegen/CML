#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/Interrupt.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
template<typename Perihperal_t> using Interrupt = soc::m4::stm32l4::Interrupt<Perihperal_t>;
#endif
} // namespace hal
} // namespace cml