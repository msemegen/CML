#pragma once

/*
    Name: String.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/numeric_traits.hpp>
#include <cml/common/cstring.hpp>
#include <cml/common/memory.hpp>
#include <cml/debug/assert.hpp>

namespace cml {
namespace collection {

class String
{
public:

    String(char* a_p_buffer, uint32 a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
        , length(0)
    {}

    String(char* a_p_buffer, uint32 a_capacity, const char* a_p_init)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
        , length(common::cstring::length(a_p_init, a_capacity))
    {
        assert(a_capacity > 0);

        common::memory::copy(this->p_buffer, a_p_init, this->length + 1);
    }

    bool push_back(char a_c)
    {
        bool retval = this->length + 1 < this->capacity;

        if (true == retval)
        {
            this->p_buffer[this->length++] = a_c;
            this->p_buffer[this->length]   = 0;
        }

        return retval;
    }

    uint32 append(const char* a_p_string, uint32 a_length)
    {
        const decltype(this->length) start = this->length;

        for (decltype(this->length) i = 0; i + this->length < this->capacity && i < a_length; i++)
        {
            this->p_buffer[this->length++] = a_p_string[i];
        }

        this->p_buffer[this->length] = 0;

        return this->length - start;
    }

    bool pop_back()
    {
        bool retval = this->length > 0;

        if (true == retval)
        {
            this->p_buffer[this->length--] = 0;
        }

        return retval;
    }

    void clear()
    {
        this->length = 0;
        this->p_buffer[0] = 0;
    }

    bool is_full() const
    {
        return this->length + 1 == this->capacity;
    };

    bool is_empty() const
    {
        return 0 == this->length;
    }

    uint32 get_length() const
    {
        return this->length;
    }

    uint32 get_capacity() const
    {
        return this->capacity;
    }

private:

    char* p_buffer;

    const uint32 capacity;
    uint32 length;
 };

} // namespace collection
} // namespace cml