#pragma once

/*
 *   Name: rcc.hpp
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
namespace m4 {
namespace stm32l4 {
template<typename Periph_t, std::size_t id = std::numeric_limits<std::size_t>::max()> class rcc
    : private cml::Non_constructible
{
};
} // namespace stm32l4
} // namespace m4
} // namespace soc