/*
    Name: Console.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <utils/Console.hpp>

//cml
#include <common/cstring.hpp>
#include <debug/assert.hpp>

namespace cml {
namespace utils {

using namespace common;

uint32 Console::write(char a_character)
{
    assert(nullptr != this->p_io_stream);

    return this->p_io_stream->transmit_bytes_polling(&a_character, 1);
}

uint32 Console::write(const char* a_p_string)
{
    assert(nullptr != this->p_io_stream);

    uint32 message_length = cstring::length(a_p_string);
    return this->p_io_stream->transmit_bytes_polling(a_p_string, message_length);
}

uint32 Console::write_line(char a_character)
{
    assert(nullptr != this->p_io_stream);

    char b[2] = { a_character, config::new_line_character };
    return this->p_io_stream->transmit_bytes_polling(b, 2);
}

uint32 Console::write_line(const char* a_p_string)
{
    assert(nullptr != this->p_io_stream);

    uint32 ret            = 0;
    uint32 message_length = cstring::length(a_p_string);

    ret += this->p_io_stream->transmit_bytes_polling(a_p_string, message_length);
    ret += this->p_io_stream->transmit_bytes_polling(&config::new_line_character, 1);

    return ret;
}

char Console::read_key()
{
    assert(nullptr != this->p_io_stream);

    char c = 0;
    this->p_io_stream->receive_bytes_polling(&c, 1);

    return c;
}

uint32 Console::read_line(char* a_p_buffer, uint32 a_max_characters_count)
{
    assert(nullptr != this->p_io_stream);

    char c = 0;
    uint32 ret = 0;
    while(ret < a_max_characters_count && config::new_line_character != a_p_buffer[ret])
    {
        this->p_io_stream->receive_bytes_polling(&c, 1);
        a_p_buffer[ret++] = c;
    }

    return ret;
}

} // namespace utils
} // namespace cml