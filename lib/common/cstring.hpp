#pragma once

/*
    Name: cstring.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>

namespace cml {
namespace common {

static inline uint32 cstring_length(const char* a_p_string, uint32 a_max_length)
{
    uint32 i = 0;
    while ((*a_p_string) != '\0' && i < a_max_length) { i++; a_p_string++; }

    return i;
}

static inline bool cstring_the_same(const char* a_p_string_1, const char* a_p_string_2, uint32 a_max_length)
{
    bool the_same = true;

    for (decltype(a_max_length) i = 0;

         i < a_max_length &&
         true == (the_same = (*a_p_string_1) == *(a_p_string_2)) &&
         0 != (*a_p_string_1) &&
         0 != (*a_p_string_2);

         i++,
         a_p_string_1++,
         a_p_string_2++);

    return the_same;
}

template<typename type>
static inline type cstring_dec_to_integer(const char* a_p_string)
{
    assert(nullptr != a_p_string && 0 != a_p_string[0]);

    type retval = 0;

    const type min = '-' == a_p_string[0] ? 0 : 1;
    const uint32 length = cstring_length(a_p_string, 22);

    for (uint32 i = length - 1, m = 1; i + length + min != length; i--, m *= 10)
    {
        assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval += (a_p_string[i] - '0') * m;
    }

    return retval * ('-' == a_p_string[0] ? -1 : 1);
}

template<typename type>
static inline char* cstring_from_dec_integer(type a_value, char* a_p_buffer, uint32 a_buffer_capacity)
{
    decltype(a_buffer_capacity) index  = 0;
    decltype(a_buffer_capacity) start  = 0;
    decltype(a_buffer_capacity) length = 0;

    type temp = a_value;

    if (a_value < 0)
    {
        assert(a_buffer_capacity > 1);

        a_p_buffer[0] = '-';
        a_p_buffer[1] = 0;
        length = 1;
        start  = 1;
    }
    else
    {
        assert(a_buffer_capacity > 0);

        a_p_buffer[index] = 0;
    }

    while (0 != temp)
    {
        char v = static_cast<char>(temp % 10);
        char to_insert = v < 0 ? v * -1 + '0' : v + '0';

        assert(to_insert >= '0' && to_insert <= '9');

        memory_copy(a_p_buffer + 1, a_p_buffer, ++length);
        a_p_buffer[start] = to_insert;

        temp /= 10;
    }

    return a_p_buffer;
}

} // namespace common
} // namespace cml