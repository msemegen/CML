#pragma once

/*
 *   Name: bit_flag.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

namespace cml {

class bit_flag
{
public:
    bit_flag()                = delete;
    bit_flag(bit_flag&&)      = delete;
    bit_flag(const bit_flag&) = delete;
    ~bit_flag()               = delete;

    bit_flag& operator=(bit_flag&&) = delete;
    bit_flag& operator=(const bit_flag&) = delete;

    template<typename Register_t, typename Flag_t> constexpr static bool is(Register_t a_register, Flag_t a_flag)
    {
        bool ret = a_flag == (a_register & a_flag);
        return ret;
    }

    template<typename Register_t, typename Mask_t> constexpr static Mask_t get(Register_t a_register, Mask_t a_mask)
    {
        return (a_register & a_mask);
    }

    template<typename Register_t, typename Flag_t> constexpr static void set(Register_t* a_p_register, Flag_t a_flag)
    {
        (*a_p_register) |= a_flag;
    }

    template<typename Register_t, typename Clear_mask_t, typename Flag_t>
    constexpr static void set(Register_t* a_p_register, Clear_mask_t a_clear_mask, Flag_t a_set_flag)
    {
        (*a_p_register) = (((*a_p_register) & (~a_clear_mask)) | a_set_flag);
    }

    template<typename Register_t, typename Flag_t> constexpr static void clear(Register_t* a_p_register, Flag_t a_flag)
    {
        (*a_p_register) &= ~a_flag;
    }
};

} // namespace cml