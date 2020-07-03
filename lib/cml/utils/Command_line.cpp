/*
    Name: Command_line.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <cml/utils/Command_line.hpp>

//cml
#include <cml/common/memory.hpp>
#include <cml/debug/assert.hpp>
#include <cml/hal/peripherals/USART.hpp>

namespace
{

using namespace cml;
using namespace cml::collection;

bool rx_callback(uint32 a_byte, bool a_idle, void* a_p_user_data)
{
    if (false == a_idle)
    {
        reinterpret_cast<Ring<char>*>(a_p_user_data)->push(static_cast<char>(a_byte));
    }

    return true;
}

} // namespace

namespace cml {
namespace utils {

using namespace cml::common;
using namespace cml::hal;

void Command_line::update()
{
    if (false == this->input_buffer_view.is_empty())
    {
        char c = this->input_buffer_view.read();

        switch (c)
        {
            case '\n':
            {
                if (this->line_length > 0)
                {
                    this->line_buffer[this->line_length] = 0;

                    this->commands_carousel.push(this->line_buffer, this->line_length);

                    this->callback_parameters_buffer_view = this->get_callback_parameters(this->line_buffer,
                                                                                          this->line_length,
                                                                                          " -",
                                                                                          2);

                    bool command_executed = this->execute_command(this->callback_parameters_buffer_view);
                    if (false == command_executed)
                    {
                        this->write_new_line();
                        this->p_io_stream->transmit_bytes_polling(this->p_command_not_found_message,
                                                                  this->command_not_found_message_length);
                    }

                    this->line_length = 0;
                }

                this->write_new_line();
                this->write_prompt();
            }
            break;

            case '\b':
            {
                if (this->line_length > 0)
                {
                    this->line_buffer[this->line_length--] = 0;
                    this->p_io_stream->transmit_bytes_polling("\b \b", 3);
                }
            }
            break;

            case '\033':
            {
                this->escape_sequence_buffer_view.push_back(c);
            }
            break;

            default:
            {
                if (false == this->escape_sequence_buffer_view.is_empty())
                {
                    this->escape_sequence_buffer_view.push_back(c);

                    if (true == this->escape_sequence_buffer_view.is_full())
                    {
                        this->execute_escape_sequence(this->escape_sequence_buffer[1],
                                                      this->escape_sequence_buffer[2]);

                        this->escape_sequence_buffer_view.clear();
                    }
                }
                else if (this->line_length + 1 < config::command_line::line_buffer_capacity)
                {
                    this->line_buffer[this->line_length++] = c;
                    this->p_io_stream->transmit_bytes_polling(&c, 1);
                }
            }
        }
    }
}

void Command_line::enable()
{
    this->p_io_stream->register_receive_callback({ rx_callback, &(this->input_buffer_view) });
}

void Command_line::disable()
{
    this->p_io_stream->unregister_receive_callback();
}

Vector<Command_line::Callback::Parameter> Command_line::get_callback_parameters(char* a_p_line,
                                                                                uint32 a_length,
                                                                                const char* a_p_separators,
                                                                                uint32 a_separators_count)
{
    Vector<Callback::Parameter> ret(this->callback_parameters_buffer,
                                    config::command_line::callback_parameters_buffer_capacity);

    auto contains = [](char a_character, const char* a_p_separators, uint32 a_separators_count)
    {
        bool ret = false;

        for (decltype(a_separators_count) i = 0; i < a_separators_count && false == ret; i++)
        {
            ret = a_p_separators[i] == a_character;
        }

        return ret;
    };

    const char* p_begin = &(a_p_line[0]);

    for (decltype(a_length) i = 0; i < a_length && false == ret.is_full(); i++)
    {
        if (nullptr != p_begin && true == contains(a_p_line[i], a_p_separators, a_separators_count))
        {
            assert(&(a_p_line[i]) > p_begin);
            ret.push_back({ p_begin, static_cast<uint32>(&(a_p_line[i]) - p_begin) });

            a_p_line[i] = 0;
            p_begin     = nullptr;
        }
        else if (nullptr == p_begin && false == contains(a_p_line[i], a_p_separators, a_separators_count))
        {
            p_begin = &(a_p_line[i]);
        }
    }

    if (nullptr != p_begin && false == ret.is_full())
    {
        assert(&(a_p_line[a_length]) > p_begin);
        ret.push_back({ p_begin, static_cast<uint32>(&(a_p_line[a_length]) - p_begin) });
    }

    return ret;
}

bool Command_line::execute_command(const Vector<Callback::Parameter>& a_parameters)
{
    uint32 index = this->callbacks_buffer_view.get_capacity();

    for (uint32 i = 0; i < this->callbacks_buffer_view.get_length() &&
                           this->callbacks_buffer_view.get_capacity() == index; i++)
    {
        if (true == cstring::equals(a_parameters[0].a_p_value,
                                    this->callbacks_buffer_view[i].p_name,
                                    a_parameters[0].length))
        {
            index = i;
        }
    }

    bool ret = index != this->callbacks_buffer_view.get_capacity();

    if (true == ret)
    {
        this->callbacks_buffer_view[index].function(a_parameters, this->callbacks_buffer_view[index].p_user_data);
    }

    return ret;
}

void Command_line::execute_escape_sequence(char a_first, char a_second)
{
    if ('[' == a_first && this->commands_carousel.get_length() > 0)
    {
        switch (a_second)
        {
            case 'A':
            {
                this->p_io_stream->transmit_bytes_polling("\033[2K\r", 5);
                this->write_prompt();

                const Commands_carousel::Command& command = this->commands_carousel.read_next();

                memory::copy(this->line_buffer, command.buffer, command.length);
                this->line_length = command.length;

                this->p_io_stream->transmit_bytes_polling(command.buffer, command.length);
            }
            break;

            case 'B':
            {
                this->p_io_stream->transmit_bytes_polling("\033[2K\r", 5);
                this->write_prompt();

                const Commands_carousel::Command& command = this->commands_carousel.read_prev();

                memory::copy(this->line_buffer, command.buffer, command.length);
                this->line_length = command.length;

                this->p_io_stream->transmit_bytes_polling(command.buffer, command.length);
            }
            break;
        }
    }
}

void Command_line::Commands_carousel::push(const char* a_p_line, uint32 a_length)
{
    if (this->write_index == config::command_line::commands_carousel_capacity)
    {
        this->write_index = 0;
    }

    this->commands[this->write_index].length = a_length;
    memory::copy(this->commands[this->write_index++].buffer, a_p_line, a_length);

    if (this->length < config::command_line::commands_carousel_capacity)
    {
        this->length++;
    }
}

const Command_line::Commands_carousel::Command& Command_line::Commands_carousel::read_next() const
{
    assert(this->length > 0);

    return this->commands[this->read_index++ % this->length];
}

const Command_line::Commands_carousel::Command& Command_line::Commands_carousel::read_prev() const
{
    assert(this->length > 0);

    return this->commands[this->read_index-- % this->length];
}

} // namespace utils
} // namespace cml