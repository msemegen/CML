#pragma once

/*
    Name: cstring.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>
#include <common/numeric_traits.hpp>

namespace cml {
namespace common {

static inline uint32 cstring_length(const char* a_p_string, uint32 a_max_length)
{
    uint32 i = 0;
    while ((*a_p_string) != '\0' && i < a_max_length) { i++; a_p_string++; }

    return i;
}

static inline uint8 cstring_to_dec_uint8(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 4) < 4);

    uint8 retval = 0;
    for (int8 i = cstring_length(a_p_string, 5) - 1, x = 1; i >= 0; i--, x *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * x;
    }

    return retval;
}

static inline int8 cstring_to_dec_int8(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 5) < 5);

    int8 retval = 0;

    int8 mul = '-' == a_p_string[0] ? -1 : 1;
    int8 min = '-' == a_p_string[0] ? 1  : 0;

    for (int8 i = cstring_length(a_p_string, 5) - 1, x = 1; i >= min; i--, x *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * x;
    }

    return retval * mul;
}

static inline uint16 cstrng_to_dec_uint16(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 6) < 6);

    uint16 retval = 0;
    for (int16 i = cstring_length(a_p_string, 6) - 1, x = 1; i >= 0; i--, x *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * x;
    }

    return retval;
}

static inline int16 cstring_to_dec_int16(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 7) < 7);

    int16 retval = 0;

    int16 mul = '-' == a_p_string[0] ? -1 : 1;
    int16 min = '-' == a_p_string[0] ? 1 : 0;

    for (int16 i = cstring_length(a_p_string, 6) - 1, x = 1; i >= min; i--, x *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * x;
    }

    return retval * mul;
}

static inline uint32 cstrng_to_dec_uint32(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 11) < 11);

    uint32 retval = 0;
    for (int32 i = cstring_length(a_p_string, 11) - 1, x = 1; i >= 0; i--, x *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * x;
    }

    return retval;
}

static inline int32 cstring_to_dec_int32(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 12) < 12);

    int32 retval = 0;

    int32 mul = '-' == a_p_string[0] ? -1 : 1;
    int32 min = '-' == a_p_string[0] ? 1 : 0;

    for (int32 i = cstring_length(a_p_string, 11) - 1, x = 1; i >= min; i--, x *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * x;
    }

    return retval * mul;
}

static inline uint64 cstrng_to_dec_uint64(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 24) < 24);

    uint64 retval = 0;
    for (int64 i = cstring_length(a_p_string, 24) - 1, x = 1; i >= 0; i--, x *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * x;
    }

    return retval;
}

static inline int64 cstring_to_dec_int64(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 25) < 25);

    int64 retval = 0;

    int64 mul = '-' == a_p_string[0] ? -1 : 1;
    int64 min = '-' == a_p_string[0] ? 1 : 0;

    for (int64 i = cstring_length(a_p_string, 25) - 1, x = 1; i >= min; i--, x *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * x;
    }

    return retval * mul;
}

static inline char* cstring_from_dec_uint8(uint8 a_value, char* a_p_buffer, uint32 a_buffer_capacity)
{
    return a_p_buffer;
}

} // namespace common
} // namespace cml