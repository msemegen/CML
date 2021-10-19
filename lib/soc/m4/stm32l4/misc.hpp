#pragma once

/*
 *   Name: misc.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// cml
#include <cml/Non_constructible.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class misc : private cml::Non_constructible
{
public:
    static void delay_us(std::uint32_t a_time);
};
} // namespace stm32l4
} // namespace m4
} // namespace soc