#pragma once

/*
    Name: wait.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// cml
#include <cml/bit_flag.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/time.hpp>

namespace cml {
namespace utils {

class wait
{
public:
    wait()            = delete;
    wait(wait&&)      = delete;
    wait(const wait&) = delete;
    ~wait()           = delete;

    wait& operator=(wait&&) = delete;
    wait& operator=(const wait&) = delete;

    template<typename Register_t> static void until(const Register_t* a_p_register, uint32_t a_flag, bool a_status)
    {
        while (a_status == cml::bit_flag::is(*a_p_register, a_flag))
            ;
    }

    template<typename Register_t> static bool
    until(const Register_t* a_p_register, uint32_t a_flag, bool a_status, time::tick a_start, time::tick a_timeout)
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
};

} // namespace utils
} // namespace cml