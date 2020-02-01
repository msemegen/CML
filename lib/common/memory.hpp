#pragma once

/*
    Name: memory.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>

namespace cml {
namespace common {

struct memory
{
    static void copy(void* a_p_destination, const void* a_p_source, uint32 a_size_in_bytes);
    static void move(void* a_p_destination, const void* a_p_source, uint32 a_size_in_bytes);
    static void set(void* a_p_destination, byte a_data, uint32 a_size_in_bytes);
    static void clear(void* a_p_destination, uint32 a_size_in_bytes);
    static bool equals(const void* a_p_first, const void* a_p_second, uint32 a_size_in_bytes);

    memory()              = delete;
    memory(memory&&)      = delete;
    memory(const memory&) = delete;
    ~memory()             = delete;

    memory& operator = (memory&&)      = delete;
    memory& operator = (const memory&) = delete;
};

} // namespace common
} // namespace cml