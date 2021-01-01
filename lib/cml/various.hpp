#pragma once

/*
    Name: various.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

#define unused(x) ((void)(x))

namespace cml {

template<class Type, uint32_t n> constexpr uint32_t get_array_length(Type (&)[n])
{
    return n;
}

} // namespace cml