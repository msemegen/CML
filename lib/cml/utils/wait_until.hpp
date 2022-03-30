#pragma once

/*
 *   Name: wait_until.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// cml
#include <cml/Duration.hpp>
#include <cml/Non_constructible.hpp>
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/utils/tick_counter.hpp>
#include <cml/various.hpp>

namespace cml {
namespace utils {
struct wait_until : private cml::Non_constructible
{
    template<typename Register_t> static void all_bits(const Register_t* a_p_register, uint32_t a_flag, bool a_status)
    {
        while (a_status == bit_flag::is(*a_p_register, a_flag))
            ;
    }

    template<typename Register_t> static bool all_bits(const Register_t* a_p_register,
                                                       uint32_t a_flag,
                                                       bool a_status,
                                                       Milliseconds a_start,
                                                       Milliseconds a_timeout)
    {
        bool status  = true;
        bool timeout = false;

        while (true == status && false == timeout)
        {
            timeout = a_timeout <= tick_counter::get() - a_start;
            status  = bit_flag::is(*a_p_register, a_flag) == a_status;
        }

        return ((false == status) && (false == timeout));
    }

    template<typename Register_t> static void any_bit(const Register_t* a_p_register, uint32_t a_flag, bool a_status)
    {
        while (a_status == bit::is_any(*a_p_register, a_flag))
            ;
    }

    template<typename Register_t> static bool any_bit(const Register_t* a_p_register,
                                                      uint32_t a_flag,
                                                      bool a_status,
                                                      Milliseconds a_start,
                                                      Milliseconds a_timeout)
    {
        bool status  = true;
        bool timeout = false;

        while (true == status && false == timeout)
        {
            timeout = a_timeout <= (tick_counter::get() - a_start);
            status  = bit::is_any(*a_p_register, a_flag) == a_status;
        }

        return ((false == status) && (false == timeout));
    }
};
} // namespace utils
} // namespace cml