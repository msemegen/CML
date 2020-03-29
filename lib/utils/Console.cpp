/*
    Name: Console.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <utils/Console.hpp>

//cml
#include <common/cstring.hpp>
#include <common/macros.hpp>
#include <debug/assert.hpp>

namespace
{

using namespace cml::collection;
using namespace cml::common;

bool rx_callback(uint32 a_byte, void* a_p_user_data)
{
    reinterpret_cast<Ring<char>*>(a_p_user_data)->push(static_cast<char>(a_byte));
    return true;
}

} // namespace

namespace cml {
namespace utils {

using namespace common;

void Console::enable()
{
    this->p_io_stream->start_read_bytes_it({ rx_callback, &(this->input_buffer_ring_view) });
}

void Console::disable()
{
    this->p_io_stream->stop_read_bytes_it();
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

char Console::read_key()
{
    assert(nullptr != this->p_io_stream);

    while (true == this->input_buffer_ring_view.is_empty());
    return this->input_buffer_ring_view.read();
}

void Console::read_line(char* a_p_buffer, uint32 a_max_characters_count)
{
    assert(nullptr != this->p_io_stream);

    for (uint32 i = 0; i < a_max_characters_count && config::new_line_character != a_p_buffer[i]; i++)
    {
        while (true == this->input_buffer_ring_view.is_empty());
        a_p_buffer[i] = this->input_buffer_ring_view.read();
    }
}

} // namespace utils
} // namespace cml