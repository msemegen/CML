#pragma once

/*
    Name: wait.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/bit.hpp>
#include <common/integer.hpp>
#include <common/time.hpp>
#include <hal/system_counter.hpp>

namespace cml {
namespace utils {

class wait
{
public:

    wait()            = delete;
    wait(wait&&)      = delete;
    wait(const wait&) = delete;
    ~wait()           = delete;

    wait& operator = (wait&&)      = delete;
    wait& operator = (const wait&) = delete;

    template<typename Register_t>
    static void until(const Register_t* a_p_register, common::uint32 a_flag, bool a_status)
    {
        while (a_status == common::is_flag(*a_p_register, a_flag));
    }

    template<typename Register_t>
    static bool until(const Register_t* a_p_register,
                      common::uint32 a_flag,
                      bool a_status,
                      common::time::tick a_start,
                      common::time::tick a_timeout)
    {
        bool status  = true;
        bool timeout = false;

        while (true == status && false == timeout)
        {
            timeout = a_timeout <= common::time::diff(hal::system_counter::get(), a_start);
            status  = common::is_flag(*a_p_register, a_flag) == a_status;
        }

        return ((false == status) && (false == timeout));
    }
};

} // namespace utils
} // namespace cml