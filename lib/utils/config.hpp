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

struct config
{
    struct console
    {
        static constexpr common::uint32 line_buffer_capacity = 128u;

        console()               = delete;
        console(console&&)      = delete;
        console(const console&) = delete;
        ~console()              = delete;

        console& operator = (console&)       = delete;
        console& operator = (const console&) = delete;
    };

    struct command_line
    {
        static constexpr common::uint32 callbacks_max_count           = 20;
        static constexpr common::uint32 callback_parameters_max_count = 4;
        static constexpr common::uint32 input_buffer_capacity         = 10;

        command_line()                    = delete;
        command_line(command_line&&)      = delete;
        command_line(const command_line&) = delete;
        ~command_line()                   = delete;

        command_line& operator = (command_line&)       = delete;
        command_line& operator = (const command_line&) = delete;
    };

    struct logger
    {
        static constexpr common::uint32 line_buffer_capacity = 128u;

        logger()              = delete;
        logger(logger&&)      = delete;
        logger(const logger&) = delete;
        ~logger()             = delete;

        logger& operator = (logger&)       = delete;
        logger& operator = (const logger&) = delete;
    };


    inline static const char new_line_character = '\n';

    config()              = delete;
    config(config&&)      = delete;
    config(const config&) = delete;
    ~config()             = delete;

    config& operator = (config&)       = delete;
    config& operator = (const config&) = delete;
};

} // namespace cml
} // namespace utils