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
        static constexpr common::uint32 LINE_BUFFER_CAPACITY = 128u;

        console()               = delete;
        console(console&&)      = delete;
        console(const console&) = delete;
        ~console()              = delete;

        console& operator = (console&)       = delete;
        console& operator = (const console&) = delete;
    };

    struct command_line
    {
        static constexpr common::uint32 CALLBACKS_MAX_COUNT           = 20;
        static constexpr common::uint32 CALLBACK_PARAMETERS_MAX_COUNT = 4;
        static constexpr common::uint32 INPUT_BUFFER_CAPACITY         = 10;

        command_line()                    = delete;
        command_line(command_line&&)      = delete;
        command_line(const command_line&) = delete;
        ~command_line()                   = delete;

        command_line& operator = (command_line&)       = delete;
        command_line& operator = (const command_line&) = delete;
    };

    inline static const char NEW_LINE_CHARACTER = '\n';

    config()              = delete;
    config(config&&)      = delete;
    config(const config&) = delete;
    ~config()             = delete;

    config& operator = (config&)       = delete;
    config& operator = (const config&) = delete;
};

} // namespace cml
} // namespace utils