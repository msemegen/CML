#pragma once

/*
    Name: various.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

namespace cml {

template<class Type, uint32_t n> uint32_t get_array_length(Type (&)[n])
{
    return n;
}

} // namespace cml