#pragma once

/*
    Name: command_line.hpp

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

class command_line
{
public:

    struct callback
    {

    };

public:

    command_line(const char* a_p_prompt, hal::c_usart* a_p_io_stream)
        : p_prompt(a_p_prompt)
        , p_io_stream(a_p_io_stream)
    {
        _assert(nullptr != this->p_prompt);
        _assert(nullptr != this->p_io_stream);

    }

    command_line()                                 = delete;
    command_line(command_line&&)                   = default;
    command_line(const command_line&)              = default;
    ~command_line()                                = default;

    command_line& operator = (command_line&&)      = default;
    command_line& operator = (const command_line&) = default;

    void start();
    void stop();
    void write_prompt();
    bool update();

    void register_callback(const callback& a_callback);

private:

    friend bool io_stream_new_byte_callback(common::byte a_byte, void* a_p_user_data);

private:

    const char* p_prompt;
    hal::c_usart* p_io_stream;

    //common::ring<common::byte, config::command_line::input_buffer_capacity> input_buffer;
    //common::vector<callback, config::command_line::callbacks_max_count>     callbacks;
};

}// namespace utils
}// namespace cml