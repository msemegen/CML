#pragma once

/*
    Name: Command_line.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>
#include <hal/usart.hpp>
#include <utils/config.hpp>

namespace cml {
namespace utils {

class Command_line
{
public:

    struct Callback
    {

    };

public:

    Command_line(const char* a_p_prompt, hal::USART* a_p_io_stream)
        : p_prompt(a_p_prompt)
        , p_io_stream(a_p_io_stream)
    {
        assert(nullptr != this->p_prompt);
        assert(nullptr != this->p_io_stream);
    }

    Command_line()                    = delete;
    Command_line(Command_line&&)      = default;
    Command_line(const Command_line&) = default;
    ~Command_line()                   = default;

    Command_line& operator = (Command_line&&)      = default;
    Command_line& operator = (const Command_line&) = default;

    void start();
    void stop();
    void write_prompt();
    bool update();

    void register_callback(const Callback& a_callback);

private:

    friend bool io_stream_new_byte_callback(common::byte a_byte, void* a_p_user_data);

private:

    const char* p_prompt;
    hal::USART* p_io_stream;
};

}// namespace utils
}// namespace cml