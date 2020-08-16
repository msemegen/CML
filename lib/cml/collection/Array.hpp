#pragma once

/*
    Name: Array.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// cml
#include <cml/debug/assert.hpp>

namespace cml {
namespace collection {

template<typename Type_t> class Array
{
public:
    Array(Type_t* a_p_buffer, uint32_t a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
    {
        for (decltype(this->capacity) i = 0; i < this->capacity; i++)
        {
            this->p_buffer[i] = Type_t();
        }
    }

    Array()                     = delete;
    Array(Array<Type_t>&&)      = default;
    Array(const Array<Type_t>&) = default;

    Array<Type_t>& operator=(Array<Type_t>&&) = default;
    Array<Type_t>& operator=(const Array<Type_t>&) = default;

    uint32_t get_capacity() const
    {
        return this->capacity;
    }

    Type_t& operator[](uint32_t a_index)
    {
        assert(a_index < this->capacity);
        return this->p_buffer[a_index];
    }

    const Type_t& operator[](uint32_t a_index) const
    {
        assert(a_index < this->capacity);
        return this->p_buffer[a_index];
    }

    bool operator==(const Array<Type_t>& a_other) const
    {
        bool retval = this->capacity == a_other.capacity;

        for (decltype(this->capacity) i = 0; i < capacity && true == retval; i++)
        {
            retval = this->p_buffer[i] = a_other.p_buffer[i];
        }

        return retval;
    }

    bool operator==(Array<Type_t>&& a_other) const
    {
        bool retval = this->capacity == a_other.capacity;

        for (decltype(this->capacity) i = 0; i < capacity && true == retval; i++)
        {
            retval = this->p_buffer[i] = a_other.p_buffer[i];
        }

        return retval;
    }

private:
    Type_t* p_buffer;
    uint32_t capacity;
};

} // namespace collection
} // namespace cml