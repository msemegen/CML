#pragma once

/*
    Name: Unbuffered_console.hpp

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

class Unbuffered_console
{
public:

    Unbuffered_console()
        : p_io_stream(nullptr)
    {}

    Unbuffered_console(hal::peripherals::USART* a_p_io_stream)
        : p_io_stream(a_p_io_stream)
    {}

    Unbuffered_console(Unbuffered_console&&)      = default;
    Unbuffered_console(const Unbuffered_console&) = default;
    ~Unbuffered_console()                         = default;

    Unbuffered_console& operator = (Unbuffered_console&&)      = default;
    Unbuffered_console& operator = (const Unbuffered_console&) = default;

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

    hal::peripherals::USART* p_io_stream;

    char line_buffer[config::console::line_buffer_capacity];
    char input_buffer[config::console::input_buffer_capacity];
};

} // namespace utils
} // namespace cml