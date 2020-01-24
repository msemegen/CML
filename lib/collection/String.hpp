#pragma once

/*
    Name: String.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/cstring.hpp>
#include <common/memory.hpp>
#include <common/numeric_traits.hpp>

namespace cml {
namespace collection {

class String
{
public:

    String(char* a_p_buffer, common::uint32 a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
        , length(0)
    {}

    String(char* a_p_buffer, common::uint32 a_capacity, const char* a_p_init)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
        , length(common::cstring_length(a_p_init, a_capacity))
    {
        assert(a_capacity > 0);

        common::memory_copy(this->p_buffer, a_p_init, this->length + 1);
    }

    void  push_back(char a_c)
    {
        assert(this->length + 1 < this->capacity);

        this->p_buffer[this->length++] = a_c;
        this->p_buffer[this->length  ] = a_c;
    }

    void pop_back()
    {
        assert(this->length > 0);

        this->p_buffer[this->length--] = 0;
    }

    void clear()
    {
        this->length = 0;
        this->p_buffer[0] = 0;
    }

    bool is_full() const
    {
        return this->length + 1 < this->capacity;
    };

    bool is_empty() const
    {
        return 0 == this->length;
    }

private:

    char* p_buffer;

    const common::uint32 capacity;
    common::uint32 length;

 };

} // namespace collection
} // namespace cml