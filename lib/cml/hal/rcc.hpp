#pragma once

/*
 *   Name: rcc.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <limits>

// soc
#ifdef STM32L4
#include <soc/m4/stm32l4/rcc.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
template<typename Perihperal_t, std::size_t id = std::numeric_limits<std::size_t>::max()> using rcc =
    soc::m4::stm32l4::rcc<Perihperal_t, id>;
#endif
} // namespace hal
} // namespace cml