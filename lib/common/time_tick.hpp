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

using time_tick = uint64;
constexpr time_tick TIME_TICK_INFINITY = Numeric_traits<time_tick>::get_max();

} // namespace common
} // namespace cml