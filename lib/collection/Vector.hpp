#pragma once

/*
    Name: Vector.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>

namespace cml {
namespace collection {

template<typename data_type>
class Vector
{
public:

    Vector(data_type* a_p_buffer, common::uint32 a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
        , length()
    {
        assert(nullptr != a_p_buffer);
        assert(0 != a_capacity);
    }

    Vector()              = default;
    Vector(Vector&&)      = default;
    Vector(const Vector&) = default;
    ~Vector()             = default;

    Vector& operator = (Vector&& a_other) = default;
    Vector& operator = (const Vector&)    = default;

    bool push_back(const data_type& a_data)
    {
        bool retval = this->length < this->capacity;

        if (true == retval)
        {
            this->p_buffer[this->length++] = a_data;
        }

        return retval;
    }

    bool pop_back()
    {
        bool retval = this->length > 0;

        if (true == retval)
        {
            this->length--;
        }

        return retval;
    }

    void clear()
    {
        this->length = 0;
    }

    data_type& operator[] (common::uint32 a_index)
    {
        assert(a_index < this->length);
        return this->p_buffer[a_index];
    }

    const data_type& operator[] (common::uint32 a_index) const
    {
        assert(a_index < this->length);
        return this->p_buffer[a_index];
    }

    common::uint32 get_length() const
    {
        return this->length;
    }

    common::uint32 get_capacity() const
    {
        return this->capacity;
    }

    bool is_empty() const
    {
        return 0 == this->length;
    }

    bool is_full() const
    {
        return this->length == this->capacity;
    }

private:

    data_type* p_buffer;
    common::uint32 capacity;

    common::uint32 length;
};

} // namespace collection
} // namespace cml