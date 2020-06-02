#pragma once

/*
    Name: Console.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/cstring.hpp>
#include <common/integer.hpp>
#include <hal/USART.hpp>
#include <utils/config.hpp>

namespace cml {
namespace utils {

class Console
{
public:

    Console()
        : p_io_stream(nullptr)
    {}

    Console(hal::USART* a_p_io_stream)
        : p_io_stream(a_p_io_stream)
    {}

    Console(Console&&)      = default;
    Console(const Console&) = default;
    ~Console()              = default;

    Console& operator = (Console&&)      = default;
    Console& operator = (const Console&) = default;

    common::uint32 write(char a_character);
    common::uint32 write(const char* a_p_string);

    template<typename ... Params_t>
    common::uint32 write(const char* a_p_format, Params_t ... a_params)
    {
        common::cstring::format(this->line_buffer, config::console::line_buffer_capacity, a_p_format, a_params ...);
        return this->write(this->line_buffer);
    }

    common::uint32 write_line(char a_character);
    common::uint32 write_line(const char* a_p_string);

    template<typename ... Params_t>
    common::uint32 write_line(const char* a_p_format, Params_t ... a_params)
    {
        common::cstring::format(this->line_buffer, config::console::line_buffer_capacity, a_p_format, a_params ...);
        return this->write_line(this->line_buffer);
    }

    char read_key();
    common::uint32 read_line(char* a_p_buffer, common::uint32 a_max_characters_count);

private:

    hal::USART* p_io_stream;

    char line_buffer[config::console::line_buffer_capacity];
};

} // namespace utils
} // namespace cml