#pragma once

/*
 *   Name: system_timer.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// cml
#include <cml/Non_constructible.hpp>

namespace soc {

class system_timer : private cml::Non_constructible
{
public:
    static uint32_t get();
    static void update();
};

} // namespace soc
