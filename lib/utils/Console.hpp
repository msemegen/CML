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

    enum Input_mode
    {
        polling,
        buffered
    };

public:

    Console(hal::USART* a_p_io_stream, Input_mode a_input_mode)
        : p_io_stream(a_p_io_stream)
        , input_buffer_ring_view(input_buffer, config::console::input_buffer_capacity)
        , input_mode(a_input_mode)
    {
        switch (this->input_mode)
        {
            case Input_mode::buffered:
            {
                this->p_read_key_function = &(read_key_buffered);
                this->p_read_line_function = &(read_line_buffered);

                this->enable_buffered_input();
            }
            break;

            case Input_mode::polling:
            {
                this->p_read_key_function = &(read_key_polling);
                this->p_read_line_function = &(read_line_polling);
            }
            break;
        }
    }

    Console()               = delete;
    Console(Console&&)      = default;
    Console(const Console&) = default;
    ~Console()              = default;

    Console& operator = (Console&&)      = default;
    Console& operator = (const Console&) = default;

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

    char read_key()
    {
        return this->p_read_key_function(this);
    }

    void read_line(char* a_p_buffer, common::uint32 a_max_characters_count)
    {
        this->p_read_line_function(this, a_p_buffer, a_max_characters_count);
    }

    bool is_enabled() const
    {
        return nullptr != this->p_io_stream;
    }

    bool is_buffered_input_enabled()
    {
        return nullptr != this->rx_interrupt_callback.p_function &&
               nullptr != this->rx_interrupt_callback.p_user_data;
    }

    Input_mode get_input_mode() const
    {
        return this->input_mode;
    }

private:

    void enable_buffered_input();

private:

    hal::USART* p_io_stream;

    hal::USART::RX_callback rx_interrupt_callback;

    char line_buffer[config::console::line_buffer_capacity];
    char input_buffer[config::console::input_buffer_capacity];

    collection::Ring<char> input_buffer_ring_view;

    Input_mode input_mode;

    char(*p_read_key_function)(Console*);
    void(*p_read_line_function)(Console*, char*, common::uint32);

private:

    static char read_key_polling(Console* a_p_this);
    static char read_key_buffered(Console* a_p_this);

    static void read_line_polling(Console* a_p_this, char* a_p_buffer, common::uint32 a_max_characters_count);
    static void read_line_buffered(Console* a_p_this, char* a_p_buffer, common::uint32 a_max_characters_count);
};

} // namespace utils
} // namespace cml