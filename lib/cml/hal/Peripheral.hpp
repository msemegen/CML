#pragma once

/*
 *   Name: Peripheral.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <limits>

#ifdef STM32L4
#include <soc/Peripheral.hpp>
#endif

namespace cml {
namespace hal {
#ifdef STM32L4
template<typename Perihperal_t,
         std::size_t id     = std::numeric_limits<std::size_t>::max(),
         typename DMA_t     = void*,
         std::size_t DMA_id = std::numeric_limits<std::size_t>::max()>
using Peripheral = soc::Peripheral<Perihperal_t, id, DMA_t, DMA_id>;
#endif
} // namespace hal
} // namespace cml