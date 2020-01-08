/*
    Name: Console.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <utils/Console.hpp>

//cml
#include <common/assert.hpp>
#include <common/cstring.hpp>

namespace cml {
namespace utils {

using namespace common;

void Console::write(char a_character)
{
    assert(nullptr != this->p_io_stream);

    this->p_io_stream->write_bytes_polling(&a_character, 1);
}

void Console::write(const char* a_p_string)
{
    assert(nullptr != this->p_io_stream);

    uint32 message_length = cstring_length(a_p_string, config::console::LINE_BUFFER_CAPACITY);
    this->p_io_stream->write_bytes_polling(a_p_string, message_length);
}

void Console::write_line(char a_character)
{
    assert(nullptr != this->p_io_stream);

    char b[2] = { a_character, config::NEW_LINE_CHARACTER };
    this->p_io_stream->write_bytes_polling(b, 2);
}

void Console::write_line(const char* a_p_string)
{
    assert(nullptr != this->p_io_stream);

    uint32 message_length = cstring_length(a_p_string, config::console::LINE_BUFFER_CAPACITY);
    this->p_io_stream->write_bytes_polling(a_p_string, message_length);
    this->p_io_stream->write_bytes_polling(&config::NEW_LINE_CHARACTER, 1);
}

char Console::read_key(bool a_echo)
{
    assert(nullptr != this->p_io_stream);

    char character = 0;
    this->p_io_stream->read_bytes_polling(&character, 1);

    if (true == a_echo)
    {
        this->p_io_stream->write_bytes_polling(&character, 1);
    }

    return character;
}

void Console::read_line(char* a_p_buffer, uint32 a_max_characters_count, bool a_echo)
{
    assert(nullptr != this->p_io_stream);
    assert(a_max_characters_count > 0);

    char character = 0;

    for (uint32 i = 0; i < a_max_characters_count && config::NEW_LINE_CHARACTER != character; i++)
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