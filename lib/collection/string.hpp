#pragma once

/*
    Name: string.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/cstring.hpp>
#include <common/numeric_traits.hpp>

namespace cml {
namespace collection {

class c_string
{
public:

    void  push_back(char c)
    {

    }

    bool is_full() const
    {
        return false;
    }

 };

} // namespace collection
} // namespace cml