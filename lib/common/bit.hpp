#pragma once

/*
    Name: bit.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>

namespace cml {
namespace common {

template<typename register_type>
bool get_bit(register_type a_register, uint8 a_index)
{
    const register_type flag = static_cast<register_type>(0x1u) << a_index;
    return flag == (a_register & flag);
}

template<typename register_type, typename flag_type>
bool is_flag(register_type a_register, flag_type a_flag)
{
    static_assert(sizeof(register_type) == sizeof(flag_type));
    return a_flag == (a_register & a_flag);
}

template<typename register_type, typename mask_type>
mask_type get_flag(register_type a_register, mask_type a_mask)
{
    static_assert(sizeof(register_type) == sizeof(mask_type));
    return (a_register & a_mask);
}

template<typename register_type>
void set_bit(register_type *a_p_register, uint8 a_index)
{
    (*a_p_register) |= (static_cast<register_type>(0x1u) << a_index);
}

template<typename register_type, typename flag_type>
void set_flag(register_type *a_p_register, flag_type a_flag)
{
    static_assert(sizeof(register_type) == sizeof(flag_type));
    (*a_p_register) |= a_flag;
}

template<typename register_type, typename clear_mask_type, typename flag_type>
void set_flag(register_type *a_p_register, clear_mask_type a_clear_mask, flag_type a_set_flag)
{
    static_assert(sizeof(register_type) == sizeof(flag_type));
    static_assert(sizeof(register_type) == sizeof(clear_mask_type));

    (*a_p_register) = (((*a_p_register) & (~a_clear_mask)) | a_set_flag);
}


template<typename register_type>
void clear_bit(register_type *a_p_register, uint8 a_index)
{
    (*a_p_register) &= ~(static_cast<register_type>(0x1u) << a_index);
}

template<typename register_type, typename flag_type>
void clear_flag(register_type *a_p_register, flag_type a_flag)
{
    (*a_p_register) &= ~a_flag;
}

template<typename register_type>
void toggle_bit(register_type* a_p_register, uint8 a_index)
{
    (*a_p_register) ^= (static_cast<register_type>(0x1u) << a_index);
}

} // namespace common
} // namespace cml