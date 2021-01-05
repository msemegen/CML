#pragma once

/*
    Name: wait_until.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/time.hpp>

namespace cml {
namespace utils {

class wait_until
{
public:
    wait_until()                  = delete;
    wait_until(wait_until&&)      = delete;
    wait_until(const wait_until&) = delete;
    ~wait_until()                 = delete;

    wait_until& operator=(wait_until&&) = delete;
    wait_until& operator=(const wait_until&) = delete;

    template<typename Register_t> static void flag(const Register_t* a_p_register, uint32_t a_flag, bool a_status)
    {
        while (a_status == cml::bit_flag::is(*a_p_register, a_flag))
            ;
    }

    template<typename Register_t> static bool
    flag(const Register_t* a_p_register, uint32_t a_flag, bool a_status, time::tick a_start, time::tick a_timeout)
    {
        bool status  = true;
        bool timeout = false;

        while (true == status && false == timeout)
        {
            timeout = a_timeout <= time::diff(hal::system_timer::get(), a_start);
            status  = cml::bit_flag::is(*a_p_register, a_flag) == a_status;
        }

        return ((false == status) && (false == timeout));
    }

    template<typename Register_t> static void any_bit(const Register_t* a_p_register, uint32_t a_flag, bool a_status)
    {
        while (a_status == cml::bit::is_any(*a_p_register, a_flag))
            ;
    }

    template<typename Register_t> static bool
    any_bit(const Register_t* a_p_register, uint32_t a_flag, bool a_status, time::tick a_start, time::tick a_timeout)
    {
        bool status  = true;
        bool timeout = false;

        while (true == status && false == timeout)
        {
            timeout = a_timeout <= time::diff(hal::system_timer::get(), a_start);
            status  = cml::bit::is_any(*a_p_register, a_flag) == a_status;
        }

        return ((false == status) && (false == timeout));
    }
};

} // namespace utils
} // namespace cml