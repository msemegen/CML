#pragma once

/*
    Name: ring_view.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>

namespace cml {
namespace common {

template<typename data_type>
class c_ring_view
{
public:

    void push(const data_type& a_data)
    {

    }

    void push(data_type&& a_data)
    {

    }

    void pop()
    {

    }

    bool is_full() const
    {
        return this->full;
    }

    bool is_empty() const
    {
        return this->head == this->tail && false == this->full;
    }

private:

    data_type* p_buffer;
    uint32 capacity;

    uint32 head;
    uint32 tail;

    bool full;
};

} // namespace common
} // namespace cml