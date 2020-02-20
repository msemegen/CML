#pragma once

/*
    Name: Array.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>
#include <debug/assert.hpp>


namespace cml {
namespace collection {

template<typename type>
class Array
{
public:

    Array(type* a_p_buffer, common::uint32 a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
    {
        for (decltype(this->capacity) i = 0; i < this->capacity; i++)
        {
            this->p_buffer[i] = type();
        }
    }

    Array()                    = delete;
    Array(Array<type>&&)       = default;
    Array(const Array<type>&&) = default;

    Array<type>& operator = (Array<type>&&)      = default;
    Array<type>& operator = (const Array<type>&) = default;

    common::uint32 get_capacity() const
    {
        return this->capacity;
    }

    type& operator[](common::uint32 a_index)
    {
        assert(a_index < this->capacity);
        return this->p_buffer[a_index];
    }

    const type& operator[](common::uint32 a_index) const
    {
        assert(a_index < this->capacity);
        return this->p_buffer[a_index];
    }

    bool operator == (const Array<type>& a_other) const
    {
        bool retval = this->capacity == a_other.capacity;

        for (decltype(this->capacity) i = 0; i < capacity && true == retval; i++)
        {
            retval = this->p_buffer[i] = a_other.p_buffer[i];
        }

        return retval;
    }

    bool operator == (Array<type>&& a_other) const
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