#pragma once

/*
 *   Name: Peripheral.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstddef>
#include <limits>

// cml
#include <cml/Non_constructible.hpp>

namespace soc {
template<typename Periph_t,
         std::size_t periph_id = std::numeric_limits<std::size_t>::max(),
         typename DMA_t        = void*,
         std::size_t DMA_id = std::numeric_limits<std::size_t>::max()>
class Peripheral
    : private cml::Non_constructible
{
};
} // namespace soc