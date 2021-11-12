#pragma once

/*
 *   Name: Factory.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/Factory.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
template<typename Perihperal_t, std::size_t id = 255u> using Factory = soc::m4::stm32l4::Factory<Perihperal_t, id>;
#endif
} // namespace hal
} // namespace cml