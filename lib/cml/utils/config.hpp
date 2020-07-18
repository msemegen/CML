#pragma once

/*
    Name: config.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

namespace cml {
namespace utils {

struct config
{
    struct console
    {
        static constexpr uint32_t line_buffer_capacity  = 128u;
        static constexpr uint32_t input_buffer_capacity = 16u;

        console()               = delete;
        console(console&&)      = delete;
        console(const console&) = delete;
        ~console()              = delete;

        console& operator = (console&)       = delete;
        console& operator = (const console&) = delete;

        static_assert(line_buffer_capacity > 1);
        static_assert(input_buffer_capacity > 1);
    };

    struct command_line
    {
        static constexpr uint32_t callbacks_buffer_capacity           = 20u;
        static constexpr uint32_t callback_parameters_buffer_capacity = 4u;
        static constexpr uint32_t input_buffer_capacity               = 16u;
        static constexpr uint32_t line_buffer_capacity                = 128u;
        static constexpr uint32_t commands_carousel_capacity          = 5u;

        command_line()                    = delete;
        command_line(command_line&&)      = delete;
        command_line(const command_line&) = delete;
        ~command_line()                   = delete;

        command_line& operator = (command_line&)       = delete;
        command_line& operator = (const command_line&) = delete;

        static_assert(callbacks_buffer_capacity > 0);
        static_assert(callback_parameters_buffer_capacity > 0);
        static_assert(input_buffer_capacity > 0);
        static_assert(line_buffer_capacity > 0);
        static_assert(commands_carousel_capacity > 0);
    };

    struct logger
    {
        static constexpr uint32_t line_buffer_capacity = 128u;

        logger()              = delete;
        logger(logger&&)      = delete;
        logger(const logger&) = delete;
        ~logger()             = delete;

        logger& operator = (logger&)       = delete;
        logger& operator = (const logger&) = delete;

        static_assert(line_buffer_capacity > 1);
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