#pragma once

/*
    Name: time_tick.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>
#include <common/Numeric_traits.hpp>

namespace cml {
namespace common {

using time_tick = uint32;
constexpr time_tick time_tick_infinity = numeric_traits<time_tick>::get_max();

static inline time_tick time_tick_diff(time_tick a_left, time_tick a_right)
{
    if (a_left < a_right)
    {
        return (time_tick_infinity - a_right) + 1 + a_left;
    }
    else
    {
        return a_left - a_right;
    }
}

} // namespace common
} // namespace cml