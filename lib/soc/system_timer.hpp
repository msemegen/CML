#pragma once

/*
 *   Name: system_timer.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

namespace soc {

class system_timer
{
public:
    static uint32_t get();
    static void update();

private:
    system_timer()                    = delete;
    system_timer(system_timer&&)      = delete;
    system_timer(const system_timer&) = delete;
    ~system_timer()                   = default;

    system_timer& operator=(system_timer&&) = delete;
    system_timer& operator=(const system_timer&) = delete;
};

} // namespace soc
