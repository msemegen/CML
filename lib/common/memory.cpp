/*
    Name: memory.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <common/assert.hpp>
#include <common/memory.hpp>

namespace cml {
namespace common {

void memory::copy(void* a_p_destination, const void* a_p_source, uint32 a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_destination);
    assert(nullptr != a_p_source);

    byte* p_destination = static_cast<byte*>(a_p_destination);
    const byte* p_source = static_cast<const byte*>(a_p_source);

    for (decltype(a_size_in_bytes) i = 0; i < a_size_in_bytes; i++)
    {
        p_destination[i] = p_source[i];
    }
}

void memory::move(void* a_p_destination, const void* a_p_source, uint32 a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_destination);
    assert(nullptr != a_p_source);

    byte* p_destination  = static_cast<byte*>(a_p_destination);
    const byte* p_source = static_cast<const byte*>(a_p_source);

    if (p_source < p_destination)
    {
        uint32 size_in_bytes = a_size_in_bytes;

        while (0 != (size_in_bytes--))
        {
            *(--p_destination) = *(--p_source);
        }
    }
    else
    {
        for (decltype(a_size_in_bytes) i = 0; i < a_size_in_bytes; i++)
        {
            p_destination[i] = p_source[i];
        }
    }
}

void memory::set(void* a_p_destination, byte a_data, uint32 a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_destination);

    byte* p_destination = static_cast<byte*>(a_p_destination);

    for (decltype(a_size_in_bytes) i = 0; i < a_size_in_bytes; i++)
    {
        p_destination[i] = a_data;
    }
}

void memory::clear(void* a_p_destination, uint32 a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_destination);

    byte* p_destination = static_cast<byte*>(a_p_destination);

    for (decltype(a_size_in_bytes) i = 0; i < a_size_in_bytes; i++)
    {
        p_destination[i] = 0x0u;
    }
}

bool memory::equals(const void* a_p_first, const void* a_p_second, uint32 a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_first);
    assert(nullptr != a_p_second);

    bool retval = true;

    const byte* p_1 = static_cast<const byte*>(a_p_first);
    const byte* p_2 = static_cast<const byte*>(a_p_second);

    for (decltype(a_size_in_bytes) i = 0; i < a_size_in_bytes && true == retval; i++)
    {
        retval = p_1[i] == p_2[i];
    }

    return retval;
}

} // namespace common
} // namespace cml