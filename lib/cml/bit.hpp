#pragma once

/*
    Name: bit.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

namespace cml {

template<typename Register_t> bool is_bit_on(Register_t a_register, uint8_t a_index)
{
    const Register_t flag = static_cast<Register_t>(0x1u) << a_index;
    return flag == (a_register & flag);
}

template<typename Register_t, typename Mask_t> bool is_any_bit_on(Register_t a_register, Mask_t a_mask)
{
    return 0 != (a_register & a_mask);
}

template<typename Register_t, typename Flag_t> bool is_flag(Register_t a_register, Flag_t a_flag)
{
    return a_flag == (a_register & a_flag);
}

template<typename Register_t, typename Mask_t> Mask_t get_flag(Register_t a_register, Mask_t a_mask)
{
    return (a_register & a_mask);
}

template<typename Register_t> void set_bit(Register_t* a_p_register, uint8_t a_index)
{
    (*a_p_register) |= (static_cast<Register_t>(0x1u) << a_index);
}

template<typename Register_t, typename Flag_t> void set_flag(Register_t* a_p_register, Flag_t a_flag)
{
    (*a_p_register) |= a_flag;
}

template<typename Register_t, typename Clear_mask_t, typename Flag_t>
void set_flag(Register_t* a_p_register, Clear_mask_t a_clear_mask, Flag_t a_set_flag)
{
    (*a_p_register) = (((*a_p_register) & (~a_clear_mask)) | a_set_flag);
}

template<typename Register_t> void clear_bit(Register_t* a_p_register, uint8_t a_index)
{
    (*a_p_register) &= ~(static_cast<Register_t>(0x1u) << a_index);
}

template<typename Register_t, typename Flag_t> void clear_flag(Register_t* a_p_register, Flag_t a_flag)
{
    (*a_p_register) &= ~a_flag;
}

template<typename Register_t> void toggle_bit(Register_t* a_p_register, uint8_t a_index)
{
    (*a_p_register) ^= (static_cast<Register_t>(0x1u) << a_index);
}

} // namespace cml