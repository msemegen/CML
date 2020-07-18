#pragma once

/*
    Name: type_traits.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

namespace cml {

#ifdef __GNUG__

template<typename Type_t>
bool is_pod()
{
    return __is_pod(Type_t);
}

#endif // GNUG

} // namespace cml