#pragma once

/*
    Name: String.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/cstring.hpp>
#include <common/numeric_traits.hpp>

namespace cml {
namespace collection {

class String
{
public:

    String(char* a_p_buffer, common::uint32 a_capacity)
        : p_buffer(a_p_buffer)
        , capacity(a_capacity)
        , length(0)
    {}

    void  push_back(char c)
    {

    }

    bool is_full() const
    {
        return false;
    };

    void clear()
    {
        this->length = 0;
        this->p_buffer[0] = 0;
    }

private:

    char* p_buffer;
    const common::uint32 capacity;
    common::uint32 length;

 };

} // namespace collection
} // namespace cml