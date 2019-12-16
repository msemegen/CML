/*
    Name: console.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <utils/console.hpp>

//cml
#include <common/assert.hpp>
#include <common/cstring.hpp>

namespace cml {
namespace utils {

using namespace common;

void c_console::write(char a_character)
{
    _assert(nullptr != this->p_io_stream);

    this->p_io_stream->write_bytes_polling(&a_character, 1);
}

void c_console::write(const char* a_p_string)
{
    _assert(nullptr != this->p_io_stream);

    uint32 message_length = cstring_length(a_p_string, s_config::s_console::line_buffer_capacity);
    this->p_io_stream->write_bytes_polling(a_p_string, message_length);
}

void c_console::write_line(char a_character)
{
    _assert(nullptr != this->p_io_stream);

    char b[2] = { a_character, s_config::new_line_character };
    this->p_io_stream->write_bytes_polling(b, 2);
}

void c_console::write_line(const char* a_p_string)
{
    _assert(nullptr != this->p_io_stream);

    uint32 message_length = cstring_length(a_p_string, s_config::s_console::line_buffer_capacity);
    this->p_io_stream->write_bytes_polling(a_p_string, message_length);
    this->p_io_stream->write_bytes_polling(&s_config::new_line_character, 1);
}

char c_console::read_key(bool a_echo)
{
    _assert(nullptr != this->p_io_stream);

    char character = 0;
    this->p_io_stream->read_bytes_polling(&character, 1);

    if (true == a_echo)
    {
        this->p_io_stream->write_bytes_polling(&character, 1);
    }

    return character;
}

void c_console::read_line(char* a_p_buffer, uint32 a_max_characters_count, bool a_echo)
{
    _assert(nullptr != this->p_io_stream);
    _assert(a_max_characters_count > 0);

    char character = 0;

    for (uint32 i = 0; i < a_max_characters_count && s_config::new_line_character != character; i++)
    {
        this->p_io_stream->read_bytes_polling(&character, 1);
        a_p_buffer[i] = character;

        if (true == a_echo)
        {
            this->p_io_stream->write_bytes_polling(&character, 1);
        }
    }
}

} // namespace utils
} // namespace cml