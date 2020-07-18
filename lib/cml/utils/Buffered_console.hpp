#pragma once

/*
    Name: Buffered_console.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

//cml
#include <cml/collection/Pair.hpp>
#include <cml/collection/Ring.hpp>
#include <cml/common/cstring.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/utils/config.hpp>

namespace cml {
namespace utils {


class Buffered_console
{
public:

    Buffered_console()
        : p_io_stream(nullptr)
        , input_buffer_view(this->input_buffer, config::console::input_buffer_capacity)
        , bus_status(hal::peripherals::USART::Bus_status_flag::ok)
    {}

    Buffered_console(hal::peripherals::USART* a_p_io_stream)
        : p_io_stream(a_p_io_stream)
        , input_buffer_view(this->input_buffer, config::console::input_buffer_capacity)
        , bus_status(hal::peripherals::USART::Bus_status_flag::ok)
    {}

    Buffered_console(Buffered_console&&)      = default;
    Buffered_console(const Buffered_console&) = default;
    ~Buffered_console()                       = default;

    Buffered_console& operator = (Buffered_console&&)      = default;
    Buffered_console& operator = (const Buffered_console&) = default;

    void enable();
    void disable();

    hal::peripherals::USART::Result write(char a_character);
    hal::peripherals::USART::Result write(const char* a_p_string);

    template<typename ... Params_t>
    hal::peripherals::USART::Result write(const char* a_p_format, Params_t ... a_params)
    {
        common::cstring::format(this->line_buffer, config::console::line_buffer_capacity, a_p_format, a_params ...);
        return this->write(this->line_buffer);
    }

    hal::peripherals::USART::Result write_line(char a_character);
    hal::peripherals::USART::Result write_line(const char* a_p_string);

    template<typename ... Params_t>
    hal::peripherals::USART::Result write_line(const char* a_p_format, Params_t ... a_params)
    {
        common::cstring::format(this->line_buffer, config::console::line_buffer_capacity, a_p_format, a_params ...);
        return this->write_line(this->line_buffer);
    }

    hal::peripherals::USART::Result read_key(char* a_p_character);
    hal::peripherals::USART::Result read_line(char* a_p_buffer, uint32_t a_buffer_size);

private:

    struct TX_context
    {
        uint32_t length        = 0;
        uint32_t index         = 0;
        volatile bool complete = false;
    };

private:

    hal::peripherals::USART* p_io_stream;

    char line_buffer[config::console::line_buffer_capacity];
    char input_buffer[config::console::input_buffer_capacity];

    collection::Ring<char> input_buffer_view;
    TX_context tx_context;

    hal::peripherals::USART::Bus_status_flag bus_status;

private:

    friend bool rx_callback(uint32_t a_byte, bool a_idle, void* a_p_user_data);
    friend bool tx_callback(volatile uint16_t* a_p_data, bool a_transfer_complete, void* a_p_user_data);
    friend bool bus_status_callback(hal::peripherals::USART::Bus_status_flag a_status, void* a_p_user_data);
};

} // namespace utils
} // namespace cml