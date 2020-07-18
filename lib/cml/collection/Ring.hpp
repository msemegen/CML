#pragma once

/*
    Name: Ring.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

//cml
#include <cml/debug/assert.hpp>

namespace cml {
namespace collection {

template<typename Type_t>
class Ring
{
public:

    Ring(Type_t* a_p_buffer, uint32_t a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
        , head(0)
        , tail(0)
        , full(false)
    {
        assert(nullptr != a_p_buffer);
        assert(0 != a_capacity);
    }

    Ring()            = delete;
    Ring(Ring&&)      = default;
    Ring(const Ring&) = default;
    ~Ring()           = default;

    Ring& operator = (Ring&&)      = default;
    Ring& operator = (const Ring&) = default;

    bool push(const Type_t& a_data)
    {
        bool add_new = false == this->is_full();

        if (true == add_new)
        {
            this->p_buffer[this->head++] = a_data;

            if (this->capacity == this->head)
            {
                this->head = 0;
            }

            this->full = this->tail == this->head;
        }

        return add_new;
    }

    const Type_t& read() const
    {
        assert(false == this->is_empty());

        const Type_t& r = this->p_buffer[this->tail++];
        this->full      = false;

        if (this->capacity == this->tail)
        {
            this->tail = 0;
        }

        return r;
    }

    void clear()
    {
        this->full = false;

        this->tail = 0;
        this->head = 0;
    }

    bool is_empty() const
    {
        return false == this->full && this->head == this->tail;
    }

    bool is_full() const
    {
        return this->full;
    }

    uint32_t get_head_index() const
    {
        return this->head;
    }

    uint32_t get_tail_index() const
    {
        return this->tail;
    }

    uint32_t get_capacity() const
    {
        return this->capacity;
    }

private:

    Type_t* p_buffer;
    uint32_t capacity;

    uint32_t head;
    mutable uint32_t tail;
    mutable bool full;
};

} // namespace collection
} // namespace cml