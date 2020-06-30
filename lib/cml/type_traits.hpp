#pragma once

namespace cml {

#ifdef __GNUG__

template<typename Type_t>
bool is_pod()
{
    return __is_pod(Type_t);
}

#endif // GNUG

} // namespace cml