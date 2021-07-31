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
#include <type_traits>

// cml
#include <cml/Non_constructible.hpp>

#define unused(x) ((void)(x))

namespace cml {

class various : private Non_constructible
{
public:
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

    template<typename Type_t> constexpr static Type_t get_enum_incorrect_value()
    {
        static_assert(true == std::is_enum_v<Type_t>);
        return static_cast<Type_t>(std::numeric_limits<std::underlying_type_t<Type_t>>::max());
    }
};

} // namespace cml

constexpr uint32_t operator"" _Hz(uint64_t a_value)
{
    return a_value;
}

constexpr uint32_t operator"" _kHz(uint64_t a_value)
{
    return a_value * 1000u;
}

constexpr uint32_t operator"" _MHz(uint64_t a_value)
{
    return a_value * 1000000u;
}
