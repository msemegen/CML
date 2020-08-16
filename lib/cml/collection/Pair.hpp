#pragma once

/*
    Name: Pair.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

namespace cml {
namespace collection {

template<typename First_t, typename Second_t> struct Pair
{
    First_t first   = First_t();
    Second_t second = Second_t();
};

} // namespace collection
} // namespace cml