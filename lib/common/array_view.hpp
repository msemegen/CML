#pragma once

/*
    Name: array_view.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>

namespace cml {
namespace common {

template<typename type>
class array_view
{
public:

    array_view(type* a_p_buffer, uint32 a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
    {
        for (decltype(this->capacity) i = 0; i < this->capacity; i++)
        {
            this->p_buffer[i] = type();
        }
    }

    array_view()                         = delete;
    array_view(array_view<type>&&)       = default;
    array_view(const array_view<type>&&) = default;

    array_view<type>& operator = (array_view<type>&&)      = default;
    array_view<type>& operator = (const array_view<type>&) = default;

    uint32 get_capacity() const
    {
        return this->capacity;
    }

    type& operator[](uint32 a_index)
    {
        _assert(a_index < this->capacity);
        return this->p_buffer[a_index];
    }

    const type& operator[](uint32 a_index) const
    {
        _assert(a_index < this->capacity);
        return this->p_buffer[a_index];
    }

    bool operator == (const array_view<type>& a_other) const
    {
        bool retval = this->capacity == a_other.capacity;

        for (decltype(this->capacity) i = 0; i < capacity && true == retval; i++)
        {
            retval = this->p_buffer[i] = a_other.p_buffer[i];
        }

        return retval;
    }

    bool operator == (array_view<type>&& a_other) const
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
    uint32 capacity;
};

} // namespace common
} // namespace cml