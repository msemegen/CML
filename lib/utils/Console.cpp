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
#include <common/macros.hpp>

namespace
{

using namespace cml::collection;
using namespace cml::common;

bool rx_callback(byte a_byte, void* a_p_user_data, bool a_timeout)
{
    unused(a_timeout);

    return reinterpret_cast<Ring<char>*>(a_p_user_data)->push(static_cast<char>(a_byte));
}

} // namespace

namespace cml {
namespace utils {

using namespace common;

void Console::enable_buffered_input()
{
    this->rx_interrupt_callback.p_user_data = &(this->input_buffer_ring_view);
    this->rx_interrupt_callback.p_function  = rx_callback;

    this->p_io_stream->read_bytes_it(this->rx_interrupt_callback, time_tick_infinity);
}

void Console::disable_buffered_input()
{
    this->rx_interrupt_callback.p_user_data = nullptr;
    this->rx_interrupt_callback.p_function  = nullptr;

    this->p_io_stream->read_bytes_it(this->rx_interrupt_callback, time_tick_infinity);
}

void Console::write(char a_character)
{
    assert(nullptr != this->p_io_stream);

    this->p_io_stream->write_bytes_polling(&a_character, 1);
}

void Console::write(const char* a_p_string)
{
    assert(nullptr != this->p_io_stream);

    uint32 message_length = cstring::length(a_p_string, config::console::line_buffer_capacity);
    this->p_io_stream->write_bytes_polling(a_p_string, message_length);
}

void Console::write_line(char a_character)
{
    assert(nullptr != this->p_io_stream);

    char b[2] = { a_character, config::new_line_character };
    this->p_io_stream->write_bytes_polling(b, 2);
}

void Console::write_line(const char* a_p_string)
{
    assert(nullptr != this->p_io_stream);

    uint32 message_length = cstring::length(a_p_string, config::console::line_buffer_capacity);
    this->p_io_stream->write_bytes_polling(a_p_string, message_length);
    this->p_io_stream->write_bytes_polling(&config::new_line_character, 1);
}

char Console::read_key(bool a_echo)
{
    assert(nullptr != this->p_io_stream);

    char character = 0;

    if (false == this->is_buffered_input())
    {
        this->p_io_stream->read_bytes_polling(&character, 1);
    }
    else
    {
        while (true == this->input_buffer_ring_view.is_empty());
        character = this->input_buffer_ring_view.read();
    }

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

    if (false == this->is_buffered_input())
    {
        for (uint32 i = 0; i < a_max_characters_count && config::new_line_character != character; i++)
        {
            this->p_io_stream->read_bytes_polling(&character, 1);
            a_p_buffer[i] = character;

            if (true == a_echo)
            {
                this->p_io_stream->write_bytes_polling(&character, 1);
            }
        }
    }
    else
    {
        for (uint32 i = 0; i < a_max_characters_count && config::new_line_character != character; i++)
        {
            while (true == this->input_buffer_ring_view.is_empty());
            a_p_buffer[i] = this->input_buffer_ring_view.read();

            if (true == a_echo)
            {
                this->p_io_stream->write_bytes_polling(&character, 1);
            }
        }
    }
}

} // namespace utils
} // namespace cml