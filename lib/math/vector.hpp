#pragma once

/*
    Name: vector.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

namespace cml {
namespace math {

template<typename type>
struct s_vector_2
{
    type x = static_cast<type>(0);
    type y = static_cast<type>(0);
};

template<typename type>
struct s_vector_3
{
    type x = static_cast<type>(0);
    type y = static_cast<type>(0);
    type z = static_cast<type>(0);
};

template<typename type>
struct s_vector_4
{
    type x = static_cast<type>(0);
    type y = static_cast<type>(0);
    type z = static_cast<type>(0);
    type w = static_cast<type>(0);
};

} // namespace math
} // namespace cml