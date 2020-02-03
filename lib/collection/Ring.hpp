#pragma once

/*
    Name: Ring.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>

namespace cml {
namespace collection {

template<typename data_type>
class Ring
{
public:

    Ring(data_type* a_p_buffer, common::uint32 a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
        , head(0)
        , tail(0)
        , full(false)
    {
        assert(nullptr != a_p_buffer);
        assert(0 != a_capacity);
    }

    bool push(const data_type& a_data)
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

    const data_type& read() const
    {
        assert(false == this->is_empty());

        const data_type& r = this->p_buffer[this->tail++];
        this->full         = false;

        if (this->capacity == this->tail)
        {
            this->tail = 0;
        }

        return r;
    }

    bool is_empty() const
    {
        return false == this->full && this->head == this->tail;
    }

    bool is_full() const
    {
        return this->full;
    }

private:

    data_type* p_buffer;
    const common::uint32 capacity;

    common::uint32 head;
    mutable common::uint32 tail;
    mutable bool full;
};

} // namespace collection
} // namespace cml