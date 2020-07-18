#pragma once

/*
    Name: time.hpp

    Copyright(c) 2019 - 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

//cml
#include <cml/numeric_traits.hpp>

namespace cml {

struct time
{
    using tick = uint32_t;

    static constexpr tick infinity = numeric_traits<tick>::get_max();

    static inline tick diff(tick a_left, tick a_right)
    {
        if (a_left < a_right)
        {
            return (infinity - a_right) + 1 + a_left;
        }
        else
        {
            return a_left - a_right;
        }
    }
};

} // namespace cml
