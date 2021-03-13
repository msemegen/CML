#pragma once

/*
 *   Name: various.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>

#define unused(x) ((void)(x))

namespace cml {

struct various
{
    various()               = delete;
    various(various&&)      = delete;
    various(const various&) = delete;
    ~various()              = delete;

    various& operator=(various&&) = delete;
    various& operator=(const various&) = delete;

    static uint32_t time_diff(uint32_t a_left, uint32_t a_right)
    {
        if (a_left < a_right)
        {
            return (std::numeric_limits<uint32_t>::max() - a_left) + 1 + a_right;
        }
        else
        {
            return a_left - a_right;
        }
    }
};

} // namespace cml
