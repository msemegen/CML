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
    {
        assert(nullptr != a_p_buffer);
        assert(0 != a_capacity);
    }

    bool push(const data_type& a_data)
    {
        return false;
    }

    data_type read() const
    {
        return this->p_buffer[0];
    }

    bool is_empty() const
    {
        return this->head == this->tail;
    }

    bool is_full() const
    {
        return false;
    }


private:

    data_type* p_buffer;
    common::uint32 capacity;

    common::uint32 head;
    common::uint32 tail;



};

} // namespace collection
} // namespace cml