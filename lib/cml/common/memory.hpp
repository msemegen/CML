#pragma once

/*
    Name: memory.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

namespace cml {
namespace common {

struct memory
{
    static void copy(void* a_p_destination, const void* a_p_source, uint32_t a_size_in_bytes);
    static void move(void* a_p_destination, const void* a_p_source, uint32_t a_size_in_bytes);
    static void set(void* a_p_destination, uint8_t a_data, uint32_t a_size_in_bytes);
    static void clear(void* a_p_destination, uint32_t a_size_in_bytes);
    static bool equals(const void* a_p_first, const void* a_p_second, uint32_t a_size_in_bytes);

    template<typename Type_t>
    static void swap(Type_t* a_p_first, Type_t* a_p_second)
    {
        Type_t tmp = *(a_p_first);

        (*a_p_first)  = *(a_p_second);
        *(a_p_second) = tmp;
    }

    memory()              = delete;
    memory(memory&&)      = delete;
    memory(const memory&) = delete;
    ~memory()             = delete;

    memory& operator = (memory&&)      = delete;
    memory& operator = (const memory&) = delete;
};

} // namespace common
} // namespace cml