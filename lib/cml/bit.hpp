#pragma once

/*
    Name: bit.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

namespace cml {

class bit
{
public:
    bit()           = delete;
    bit(bit&&)      = delete;
    bit(const bit&) = delete;
    ~bit()          = delete;

    bit& operator=(bit&&) = delete;
    bit& operator=(const bit&) = delete;

    template<typename Register_t> constexpr static bool is(Register_t a_register, uint8_t a_index)
    {
        const Register_t flag = static_cast<Register_t>(0x1u) << a_index;
        return flag == (a_register & flag);
    }

    template<typename Register_t, typename Mask_t> constexpr static bool is_any(Register_t a_register, Mask_t a_mask)
    {
        return 0 != (a_register & a_mask);
    }

    template<typename Register_t> constexpr static void set(Register_t* a_p_register, uint8_t a_index)
    {
        (*a_p_register) |= (static_cast<Register_t>(0x1u) << a_index);
    }

    template<typename Register_t> constexpr static void clear(Register_t* a_p_register, uint8_t a_index)
    {
        (*a_p_register) &= ~(static_cast<Register_t>(0x1u) << a_index);
    }

    template<typename Register_t> constexpr static void toggle(Register_t* a_p_register, uint8_t a_index)
    {
        (*a_p_register) ^= (static_cast<Register_t>(0x1u) << a_index);
    }
};

} // namespace cml