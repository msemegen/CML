#pragma once

/*
    Name: config.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>

namespace cml {
namespace utils {

struct s_config
{
    struct s_console
    {
        static constexpr common::uint32 line_buffer_capacity = 128u;

        s_console()                 = delete;
        s_console(s_console&&)      = delete;
        s_console(const s_console&) = delete;
        ~s_console()                = delete;

        s_console& operator = (s_console&)       = delete;
        s_console& operator = (const s_console&) = delete;
    };

    struct s_command_line
    {
        static constexpr common::uint32 callbacks_max_count           = 20;
        static constexpr common::uint32 callback_parameters_max_count = 4;
        static constexpr common::uint32 input_buffer_capacity         = 10;

        s_command_line()                      = delete;
        s_command_line(s_command_line&&)      = delete;
        s_command_line(const s_command_line&) = delete;
        ~s_command_line()                     = delete;

        s_command_line& operator = (s_command_line&)       = delete;
        s_command_line& operator = (const s_command_line&) = delete;
    };

    inline static const char new_line_character = '\n';

    s_config()                = delete;
    s_config(s_config&&)      = delete;
    s_config(const s_config&) = delete;
    ~s_config()               = delete;

    s_config& operator = (s_config&)       = delete;
    s_config& operator = (const s_config&) = delete;
};

} // namespace cml
} // namespace utils