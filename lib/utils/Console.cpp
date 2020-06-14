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

namespace {

using namespace cml::collection;
using namespace cml::common;

bool rx_callback(uint32 a_byte, bool a_idle, void* a_p_user_data)
{
    if (false == a_idle)
    {
        reinterpret_cast<Ring<char>*>(a_p_user_data)->push(static_cast<char>(a_byte));
    }

    return true;
}

} // namespace ::

namespace cml {
namespace utils {

using namespace common;

void Console::enable_buffered_input()
{
    this->p_io_stream->register_receive_callback({ rx_callback, &(this->input_buffer_view) });
    this->buffered_input_enabled = true;
}

void Console::disable_buffered_input()
{
    this->p_io_stream->unregister_receive_callback();
    this->buffered_input_enabled = false;
}

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

    if (false == this->buffered_input_enabled)
    {
        this->p_io_stream->receive_bytes_polling(&c, 1);
    }
    else
    {
        while(true == this->input_buffer_view.is_empty());
        c = this->input_buffer_view.read();
    }

    return c;
}

uint32 Console::read_line(char* a_p_buffer, uint32 a_max_characters_count)
{
    assert(nullptr != this->p_io_stream);

    uint32 ret = 0;

    while (ret < a_max_characters_count && config::new_line_character != a_p_buffer[ret])
    {
        a_p_buffer[ret++] = this->read_key();
    }

    return ret;
}

} // namespace utils
} // namespace cml