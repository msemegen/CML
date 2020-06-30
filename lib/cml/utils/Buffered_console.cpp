/*
    Name: Buffered_console.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <cml/utils/Buffered_console.hpp>

//cml
#include <cml/common/cstring.hpp>
#include <cml/common/memory.hpp>
#include <cml/debug/assert.hpp>

namespace cml {
namespace utils {

using namespace cml::collection;
using namespace cml::common;
using namespace cml::hal::peripherals;

bool rx_callback(uint32 a_byte, bool a_idle, void* a_p_user_data)
{
    if (false == a_idle)
    {
        reinterpret_cast<Ring<char>*>(a_p_user_data)->push(static_cast<char>(a_byte));
    }

    return true;
}

bool tx_callback(volatile uint16* a_p_data, bool a_transfer_complete, void* a_p_user_data)
{
    Buffered_console* p_this = reinterpret_cast<Buffered_console*>(a_p_user_data);

    if (false == a_transfer_complete)
    {
        if (p_this->tx_context.index < p_this->tx_context.length)
        {
            (*a_p_data) = p_this->line_buffer[p_this->tx_context.index++];
            return true;
        }
    }
    else
    {
        p_this->p_io_stream->unregister_transmit_callback();
        return false;
    }

    return true;
}

bool bus_status_callback(USART::Bus_status_flag a_status, void* a_p_user_data)
{
    Buffered_console* p_this = reinterpret_cast<Buffered_console*>(a_p_user_data);

    if (true == p_this->p_io_stream->is_transmit_callback_registered())
    {
        p_this->p_io_stream->unregister_transmit_callback();
    }

    p_this->bus_status = a_status;

    return true;
}

void Buffered_console::enable()
{
    assert(nullptr != this->p_io_stream);

    this->p_io_stream->register_receive_callback({ rx_callback, &(this->input_buffer_view) });
    this->p_io_stream->register_bus_status_callback({ bus_status_callback, this });
}

void Buffered_console::disable()
{
    assert(nullptr != this->p_io_stream);

    this->p_io_stream->unregister_receive_callback();
    this->p_io_stream->unregister_bus_status_callback();
    this->p_io_stream->unregister_transmit_callback();
}

USART::Result Buffered_console::write(char a_character)
{
    assert(nullptr != this->p_io_stream);
    assert(false == this->p_io_stream->is_transmit_callback_registered());
    assert(true == this->p_io_stream->is_bus_status_callback_registered());

    USART::Result ret;

    this->tx_context.index  = 0;
    this->tx_context.length = 1;

    this->line_buffer[0] = a_character;

    this->p_io_stream->register_transmit_callback({ tx_callback, this });
    while (true == this->p_io_stream->is_transmit_callback_registered());

    ret = { this->bus_status, this->tx_context.index };

    if (USART::Bus_status_flag::ok != this->bus_status)
    {
        this->bus_status = USART::Bus_status_flag::ok;
    }

    return ret;
}

USART::Result Buffered_console::write(const char* a_p_string)
{
    assert(nullptr != a_p_string);
    assert(nullptr != this->p_io_stream);
    assert(false == this->p_io_stream->is_transmit_callback_registered());
    assert(true == this->p_io_stream->is_bus_status_callback_registered());

    USART::Result ret;

    this->tx_context.index  = 0;
    this->tx_context.length = cstring::length(a_p_string);

    memory::copy(this->line_buffer, a_p_string, this->tx_context.length);

    this->p_io_stream->register_transmit_callback({ tx_callback, this });
    while (true == this->p_io_stream->is_transmit_callback_registered());

    ret = { this->bus_status, this->tx_context.index };

    if (USART::Bus_status_flag::ok != this->bus_status)
    {
        this->bus_status = USART::Bus_status_flag::ok;
    }

    return ret;
}

USART::Result Buffered_console::write_line(char a_character)
{
    assert(nullptr != this->p_io_stream);
    assert(false == this->p_io_stream->is_transmit_callback_registered());
    assert(true == this->p_io_stream->is_bus_status_callback_registered());

    USART::Result ret;

    this->tx_context.index  = 0;
    this->tx_context.length = 2;

    this->line_buffer[0] = a_character;
    this->line_buffer[1] = config::new_line_character;

    this->p_io_stream->register_transmit_callback({ tx_callback, this });

    while (true == this->p_io_stream->is_transmit_callback_registered());

    ret = { this->bus_status, this->tx_context.index };

    if (USART::Bus_status_flag::ok != this->bus_status)
    {
        this->bus_status = USART::Bus_status_flag::ok;
    }

    return ret;
}

USART::Result Buffered_console::write_line(const char* a_p_string)
{
    assert(nullptr != a_p_string);
    assert(nullptr != this->p_io_stream);
    assert(false == this->p_io_stream->is_transmit_callback_registered());
    assert(true == this->p_io_stream->is_bus_status_callback_registered());

    USART::Result ret;

    this->tx_context.index  = 0;
    this->tx_context.length = cstring::length(a_p_string, config::console::line_buffer_capacity - 1);

    memory::copy(this->line_buffer, a_p_string, this->tx_context.length);
    this->line_buffer[this->tx_context.length++] = config::new_line_character;

    this->p_io_stream->register_transmit_callback({ tx_callback, this });
    while (true == this->p_io_stream->is_transmit_callback_registered());

    ret = { this->bus_status, this->tx_context.index };

    if (USART::Bus_status_flag::ok != this->bus_status)
    {
        this->bus_status = USART::Bus_status_flag::ok;
    }

    return ret;
}

USART::Result Buffered_console::read_key(char* a_p_character)
{
    assert(nullptr != a_p_character);
    assert(nullptr != this->p_io_stream);

    assert(false == this->p_io_stream->is_receive_callback_registered());
    assert(true == this->p_io_stream->is_bus_status_callback_registered());

    USART::Result ret;

    while(true == this->input_buffer_view.is_empty() && USART::Bus_status_flag::ok == this->bus_status);

    ret = { this->bus_status, 0 };

    if (false == this->input_buffer_view.is_empty() && USART::Bus_status_flag::ok == this->bus_status)
    {
        ret.data_length = 1;
        (*a_p_character) = this->input_buffer_view.read();
    }

    if (USART::Bus_status_flag::ok != this->bus_status)
    {
        this->bus_status = USART::Bus_status_flag::ok;
    }

    return ret;
}

USART::Result Buffered_console::read_line(char* a_p_buffer, uint32 a_buffer_size)
{
    assert(nullptr != a_p_buffer);

    USART::Result ret;
    USART::Result tmp;

    do
    {
        tmp = this->read_key(&(a_p_buffer[ret.data_length]));

        ret.data_length += tmp.data_length;
        ret.bus_status = tmp.bus_status;
    }
    while (ret.data_length < a_buffer_size &&
           ret.bus_status == USART::Bus_status_flag::ok &&
           config::new_line_character != a_p_buffer[ret.data_length - 1]);

    return ret;
}

} // namespace utils
} // namespace cml