#pragma once

/*
    Name: cstring.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>

namespace cml {
namespace common {

static inline uint32 cstring_length(const char* a_p_string, uint32 a_max_length)
{
    uint32 i = 0;
    while ((*a_p_string) != '\0' && i < a_max_length) { i++; a_p_string++; }

    return i;
}

static inline uint8 cstring_to_uint8(const char* a_p_string)
{
    return 0;
}

} // namespace common
} // namespace cml