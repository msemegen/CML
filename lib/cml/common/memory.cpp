/*
    Name: memory.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <cml/common/memory.hpp>
#include <cml/debug/assert.hpp>

namespace cml {
namespace common {

void memory::copy(void* a_p_destination, const void* a_p_source, uint32_t a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_destination);
    assert(nullptr != a_p_source);

    uint8_t* p_destination  = static_cast<uint8_t*>(a_p_destination);
    const uint8_t* p_source = static_cast<const uint8_t*>(a_p_source);

    for (decltype(a_size_in_bytes) i = 0; i < a_size_in_bytes; i++)
    {
        p_destination[i] = p_source[i];
    }
}

void memory::move(void* a_p_destination, const void* a_p_source, uint32_t a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_destination);
    assert(nullptr != a_p_source);

    uint8_t* p_destination  = static_cast<uint8_t*>(a_p_destination);
    const uint8_t* p_source = static_cast<const uint8_t*>(a_p_source);

    if (p_source < p_destination)
    {
        uint32_t size_in_bytes = a_size_in_bytes;

        p_destination += size_in_bytes;
        p_source      += size_in_bytes;

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

void memory::set(void* a_p_destination, uint8_t a_data, uint32_t a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_destination);

    uint8_t* p_destination = static_cast<uint8_t*>(a_p_destination);

    for (decltype(a_size_in_bytes) i = 0; i < a_size_in_bytes; i++)
    {
        p_destination[i] = a_data;
    }
}

void memory::clear(void* a_p_destination, uint32_t a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_destination);

    uint8_t* p_destination = static_cast<uint8_t*>(a_p_destination);

    for (decltype(a_size_in_bytes) i = 0; i < a_size_in_bytes; i++)
    {
        p_destination[i] = 0x0u;
    }
}

bool memory::equals(const void* a_p_first, const void* a_p_second, uint32_t a_size_in_bytes)
{
    assert(a_size_in_bytes > 0);
    assert(nullptr != a_p_first);
    assert(nullptr != a_p_second);

    bool retval = true;

    const uint8_t* p_1 = static_cast<const uint8_t*>(a_p_first);
    const uint8_t* p_2 = static_cast<const uint8_t*>(a_p_second);

    for (decltype(a_size_in_bytes) i = 0; i < a_size_in_bytes && true == retval; i++)
    {
        retval = p_1[i] == p_2[i];
    }

    return retval;
}

} // namespace common
} // namespace cml