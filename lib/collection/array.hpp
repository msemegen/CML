#pragma once

/*
    Name: array.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>

namespace cml {
namespace collection {

template<typename type>
class c_array
{
public:

    c_array(type* a_p_buffer, common::uint32 a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
    {
        for (decltype(this->capacity) i = 0; i < this->capacity; i++)
        {
            this->p_buffer[i] = type();
        }
    }

    c_array()                      = delete;
    c_array(c_array<type>&&)       = default;
    c_array(const c_array<type>&&) = default;

    c_array<type>& operator = (c_array<type>&&)      = default;
    c_array<type>& operator = (const c_array<type>&) = default;

    common::uint32 get_capacity() const
    {
        return this->capacity;
    }

    type& operator[](common::uint32 a_index)
    {
        _assert(a_index < this->capacity);
        return this->p_buffer[a_index];
    }

    const type& operator[](common::uint32 a_index) const
    {
        _assert(a_index < this->capacity);
        return this->p_buffer[a_index];
    }

    bool operator == (const c_array<type>& a_other) const
    {
        bool retval = this->capacity == a_other.capacity;

        for (decltype(this->capacity) i = 0; i < capacity && true == retval; i++)
        {
            retval = this->p_buffer[i] = a_other.p_buffer[i];
        }

        return retval;
    }

    bool operator == (c_array<type>&& a_other) const
    {
        bool retval = this->capacity == a_other.capacity;

        for (decltype(this->capacity) i = 0; i < capacity && true == retval; i++)
        {
            retval = this->p_buffer[i] = a_other.p_buffer[i];
        }

        return retval;
    }

private:

    type* p_buffer;
    common::uint32 capacity;
};

} // namespace collection
} // namespace cml