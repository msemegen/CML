#pragma once

/*
    Name: Console.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <collection/Ring.hpp>
#include <common/cstring.hpp>
#include <common/integer.hpp>
#include <hal/USART.hpp>
#include <utils/config.hpp>

namespace cml {
namespace utils {

class Console
{
public:

    Console(hal::USART* a_p_io_stream, bool a_buffered_input)
        : p_io_stream(a_p_io_stream)
        , input_buffer_ring_view(input_buffer, config::console::input_buffer_capacity)
    {}

    Console()               = delete;
    Console(Console&&)      = default;
    Console(const Console&) = default;
    ~Console()              = default;

    Console& operator = (Console&&)      = default;
    Console& operator = (const Console&) = default;

    void enable_buffered_input();
    void disable_buffered_input();

    void write(char a_character);
    void write(const char* a_p_string);

    template<typename ... params>
    void write(const char* a_p_format, params ... a_params)
    {
        common::cstring::format(this->line_buffer, config::console::line_buffer_capacity, a_p_format, a_params ...);
        this->write(this->line_buffer);
    }

    void write_line(char a_character);
    void write_line(const char* a_p_string);

    template<typename ... params>
    void write_line(const char* a_p_format, params ... a_params)
    {
        common::cstring::format(this->line_buffer, config::console::line_buffer_capacity, a_p_format, a_params ...);
        this->write_line(this->line_buffer);
    }

    char read_key(bool a_echo);
    void read_line(char* a_p_buffer, common::uint32 a_max_characters_count, bool a_echo);

    bool is_enabled() const
    {
        return nullptr != this->p_io_stream;
    }

    bool is_buffered_input()
    {
        return nullptr != this->rx_interrupt_callback.p_function &&
               nullptr != this->rx_interrupt_callback.p_user_data;
    }

private:

    hal::USART* p_io_stream;

    hal::USART::RX_callback rx_interrupt_callback;

    char line_buffer[config::console::line_buffer_capacity];
    char input_buffer[config::console::input_buffer_capacity];

    collection::Ring<char> input_buffer_ring_view;
};

} // namespace utils
} // namespace cml