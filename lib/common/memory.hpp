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

void memory_copy(void* a_p_destination, const void* a_p_source, uint32 a_size_in_bytes);
void memory_set(void* a_p_destination, byte a_data, uint32 a_size_in_bytes);
void memory_clear(void* a_p_destination, uint32 a_size_in_bytes);
bool memory_equals(const void* a_p_first, const void* a_p_second, uint32 a_size_in_bytes);

} // namespace common
} // namespace cml