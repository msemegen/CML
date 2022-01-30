#pragma once

/*
 *   Name: ms_tick_counter.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// cml
#include <cml/Non_constructible.hpp>

namespace cml {
namespace utils {
class ms_tick_counter : private cml::Non_constructible
{
public:
    static std::uint32_t get();
    static void update();
    static void update(void*);
};
} // namespace utils
} // namespace cml
