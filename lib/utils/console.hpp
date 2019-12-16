#pragma once

/*
    Name: console.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/format.hpp>
#include <common/integer.hpp>
#include <common/string_view.hpp>
#include <hal/usart.hpp>
#include <utils/config.hpp>


namespace cml {
namespace utils {

class c_console
{
public:

    c_console(hal::c_usart* a_p_io_stream)
        : p_io_stream(a_p_io_stream)
        , line_buffer_view(this->line_buffer, s_config::s_console::line_buffer_capacity)
    {}

    c_console()                 = delete;
    c_console(c_console&&)      = default;
    c_console(const c_console&) = default;
    ~c_console()                = default;

    c_console& operator = (c_console&&)      = default;
    c_console& operator = (const c_console&) = default;

    void write(char a_character);
    void write(const char* a_p_string);

    template<typename ... params>
    void write(const char* a_p_format, params ... a_params)
    {
        this->line_buffer_view.clear();
        common::c_format(&(this->line_buffer_view), a_p_format, a_params ...);
        this->write(this->line_buffer);
    }

    void write_line(char a_character);
    void write_line(const char* a_p_string);

    template<typename ... params>
    void write_line(const char* a_p_format, params ... a_params)
    {
        this->line_buffer_view.clear();
        common::c_format(&(this->line_buffer_view), a_p_format, a_params ...);
        this->write_line(this->line_buffer);
    }

    char read_key(bool a_echo);
    void read_line(char* a_p_buffer, common::uint32 a_max_characters_count, bool a_echo);

    bool is_enabled() const
    {
        return nullptr != this->p_io_stream;
    }

private:

    hal::c_usart* p_io_stream;

    char line_buffer[s_config::s_console::line_buffer_capacity];
    common::c_string_view line_buffer_view;
};

} // namespace utils
} // namespace cml