#pragma once

/*
 *   Name: misc.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

namespace soc {
namespace m4 {
namespace stm32l4 {

class misc
{
public:
    misc()            = delete;
    misc(misc&&)      = delete;
    misc(const misc&) = delete;

    misc& operator=(misc&&) = delete;
    misc& operator=(const misc&) = delete;

    static void delay_us(uint32_t a_time);
};

} // namespace stm32l4
} // namespace m4
} // namespace soc