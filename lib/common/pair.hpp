#pragma once

/*
    Name: pair.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

namespace cml {
namespace container {

template<typename type_a, typename type_b = type_a>
struct pair
{
    type_a a = type_a();
    type_b b = type_b();
};

} // namespace container
} // namespace cml