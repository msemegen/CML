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

static inline uint8 cstring_dec_to_uint8(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 4) < 4); //max: strlen("255") + 1

    uint8 retval      = 0;
    const uint32 length = cstring_length(a_p_string, 3);

    for (uint32 i = length - 1, m = 1; i + length + 1 != length; i--, m *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * m;
    }

    return retval;
}

static inline int8 cstring_dec_to_int8(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 5) < 5); //max: strlen("-255") + 1

    int8 retval = 0;

    const uint32 min = '-' == a_p_string[0] ? 0 : 1;
    const uint32 length = cstring_length(a_p_string, 4);

    for (uint32 i = length - 1, m = 1; i + length + min != length; i--, m *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * m;
    }

    return retval * ('-' == a_p_string[0] ? -1 : 1);
}

static inline uint16 cstrng_dec_to_uint16(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 6) < 6); //max: strlen("65536") + 1

    uint16 retval = 0;
    const uint32 length = cstring_length(a_p_string, 5);

    for (uint32 i = length - 1, m = 1; i + length + 1 != length; i--, m *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * m;
    }

    return retval;
}

static inline int16 cstring_dec_to_int16(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 7) < 7); //max: strlen("-65535") + 1

    int16 retval = 0;

    const uint32 min    = '-' == a_p_string[0] ? 0 : 1;
    const uint32 length = cstring_length(a_p_string, 4);

    for (uint32 i = length - 1, m = 1; i + length + min != length; i--, m *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * m;
    }

    return retval * ('-' == a_p_string[0] ? -1 : 1);
}

static inline uint32 cstrng_dec_to_uint32(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 11) < 11); //max: strlen("4294967295") + 1

    uint32 retval = 0;
    const uint32 length = cstring_length(a_p_string, 10);

    for (uint32 i = length - 1, m = 1; i + length + 1 != length; i--, m *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * m;
    }

    return retval;
}

static inline int32 cstring_to_dec_int32(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 12) < 12); // max: strlen("-2147483648") + 1

    int32 retval = 0;

    const uint32 min    = '-' == a_p_string[0] ? 0 : 1;
    const uint32 length = cstring_length(a_p_string, 12);

    for (uint32 i = length - 1, m = 1; i + length + min != length; i--, m *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * m;
    }

    return retval * ('-' == a_p_string[0] ? -1 : 1);
}

static inline uint64 cstrng_dec_to_uint64(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 21) < 21); //max: strlen("18446744073709551615") + 1

    uint64 retval = 0;
    const uint32 length = cstring_length(a_p_string, 20);

    for (uint32 i = length - 1, m = 1; i + length + 1 != length; i--, m *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * m;
    }

    return retval;
}

static inline int64 cstring_to_dec_int64(const char* a_p_string)
{
    _assert(nullptr != a_p_string && 0 != a_p_string[0]);
    _assert(cstring_length(a_p_string, 22) < 22); // max: strlen("-9223372036854775808") + 1

    int64 retval = 0;

    const uint32 min    = '-' == a_p_string[0] ? 0 : 1;
    const uint32 length = cstring_length(a_p_string, 22);

    for (uint32 i = length - 1, m = 1; i + length + min != length; i--, m *= 10)
    {
        _assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * m;
    }

    return retval * ('-' == a_p_string[0] ? -1 : 1);
}

static inline char* cstring_from_dec_uint8(uint8 a_value, char* a_p_buffer, uint32 a_buffer_capacity)
{
    return a_p_buffer;
}

} // namespace common
} // namespace cml