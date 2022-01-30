#pragma once

/*
 *   Name: DMA.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4
#include <soc/m4/stm32l4/DMA.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
template<typename Periph_t = void*> using DMA = soc::m4::stm32l4::DMA<Periph_t>;
#endif
} // namespace hal
} // namespace cml