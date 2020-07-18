/*
    Name: Unbuffered_console.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <cml/utils/Unbuffered_console.hpp>

//cml
#include <cml/common/cstring.hpp>
#include <cml/common/memory.hpp>
#include <cml/debug/assert.hpp>

namespace cml {
namespace utils {

using namespace cml::collection;
using namespace cml::common;
using namespace cml::hal::peripherals;

USART::Result Unbuffered_console::write(char a_character)
{
    assert(nullptr != this->p_io_stream);

    return this->p_io_stream->transmit_bytes_polling(&a_character, 1);
}

USART::Result Unbuffered_console::write(const char* a_p_string)
{
    assert(nullptr != this->p_io_stream);

    uint32_t message_length = cstring::length(a_p_string);
    return this->p_io_stream->transmit_bytes_polling(a_p_string, message_length);
}

USART::Result Unbuffered_console::write_line(char a_character)
{
    assert(nullptr != this->p_io_stream);

    char b[2] = { a_character, config::new_line_character };
    return this->p_io_stream->transmit_bytes_polling(b, 2);
}

USART::Result Unbuffered_console::write_line(const char* a_p_string)
{
    assert(nullptr != this->p_io_stream);
    assert(nullptr != a_p_string);

    uint32_t message_length = cstring::length(a_p_string);

    USART::Result ret = this->p_io_stream->transmit_bytes_polling(a_p_string, message_length);

    if (USART::Bus_status_flag::ok == ret.bus_status)
    {
        auto [status, data_length] = this->p_io_stream->transmit_bytes_polling(&config::new_line_character, 1);
        ret.data_length_in_words += data_length;
        ret.bus_status = status;
    }

    return ret;
}

USART::Result Unbuffered_console::read_key(char* a_p_character)
{
    assert(nullptr != this->p_io_stream);
    assert(nullptr != a_p_character);

    return this->p_io_stream->receive_bytes_polling(a_p_character, 1);
}

USART::Result Unbuffered_console::read_line(char* a_p_buffer, uint32_t a_buffer_size)
{
    assert(nullptr != a_p_buffer);
    assert(a_buffer_size > 0);
    assert(nullptr != this->p_io_stream);

    USART::Result ret;
    USART::Result tmp;

    do
    {
        tmp = this->p_io_stream->receive_bytes_polling(&(a_p_buffer[ret.data_length_in_words]), 1);

        ret.data_length_in_words += tmp.data_length_in_words;
        ret.bus_status = tmp.bus_status;
    }
    while (ret.data_length_in_words < a_buffer_size &&
           ret.bus_status == USART::Bus_status_flag::ok &&
           config::new_line_character != a_p_buffer[ret.data_length_in_words - 1]);

    return ret;
}

} // namespace utils
} // namespace cml