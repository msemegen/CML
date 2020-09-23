/*
    Name: Command_line.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <cml/utils/Command_line.hpp>

// cml
#include <cml/common/memory.hpp>
#include <cml/debug/assert.hpp>
#include <cml/hal/peripherals/USART.hpp>

namespace cml {
namespace utils {

using namespace cml::common;
using namespace cml::hal;

void Command_line::update()
{
    char c[]        = { 0, 0, 0 };
    uint32_t length = this->read_character.function(c, sizeof(c), this->read_character.p_user_data);
    if (1 == length)
    {
        switch (c[0])
        {
            case '\n': {
                if (this->line_length > 0)
                {
                    this->line_buffer[this->line_length] = 0;

                    this->commands_carousel.push(this->line_buffer, this->line_length);

                    this->get_callback_parameters(this->callback_parameters_buffer,
                                                  config::command_line::callback_parameters_buffer_capacity,
                                                  &(this->callback_parameters_buffer_length),
                                                  this->line_buffer,
                                                  this->line_length,
                                                  " -",
                                                  2);

                    bool command_executed = this->execute_command(this->callback_parameters_buffer,
                                                                  this->callback_parameters_buffer_length);
                    if (false == command_executed)
                    {
                        this->write_new_line();
                        this->write_string.function(this->p_command_not_found_message,
                                                    this->command_not_found_message_length,
                                                    this->write_string.p_user_data);
                    }

                    this->line_length = 0;
                }

                this->write_new_line();
                this->write_prompt();
            }
            break;

            case '\b': {
                if (this->line_length > 0)
                {
                    this->line_buffer[this->line_length--] = 0;
                    this->write_string.function("\b \b", 3, this->write_string.p_user_data);
                }
            }
            break;

            default: {
                if (this->line_length + 1 < config::command_line::line_buffer_capacity)
                {
                    this->line_buffer[this->line_length++] = c[0];
                    this->write_character.function(c[0], this->write_character.p_user_data);
                }
            }
        }
    }
    else if (3 == length && '\033' == c[0])
    {
        this->execute_escape_sequence(c[1], c[2]);
    }
}

void Command_line::get_callback_parameters(Callback::Parameter* a_p_out_buffer,
                                           uint32_t a_out_buffor_capacity,
                                           uint32_t* a_p_out_buffer_length,
                                           char* a_p_line,
                                           uint32_t a_line_length,
                                           const char* a_p_separators,
                                           uint32_t a_separators_count)
{
    assert(nullptr != a_p_out_buffer);
    assert(nullptr != a_p_line);
    assert(nullptr != a_p_out_buffer_length);
    assert(nullptr != a_p_separators);

    auto contains = [](char a_character, const char* a_p_separators, uint32_t a_separators_count) {
        bool ret = false;

        for (decltype(a_separators_count) i = 0; i < a_separators_count && false == ret; i++)
        {
            ret = a_p_separators[i] == a_character;
        }

        return ret;
    };

    const char* p_begin      = &(a_p_line[0]);
    (*a_p_out_buffer_length) = 0;

    for (uint32_t i = 0;
         i < a_line_length && (*a_p_out_buffer_length) < config::command_line::callback_parameters_buffer_capacity;
         i++)
    {
        if (nullptr != p_begin && true == contains(a_p_line[i], a_p_separators, a_separators_count))
        {
            assert(&(a_p_line[i]) > p_begin);
            a_p_out_buffer[(*a_p_out_buffer_length)++] = { p_begin, static_cast<uint32_t>(&(a_p_line[i]) - p_begin) };

            a_p_line[i] = 0;
            p_begin     = nullptr;
        }
        else if (nullptr == p_begin && false == contains(a_p_line[i], a_p_separators, a_separators_count))
        {
            p_begin = &(a_p_line[i]);
        }
    }

    if (nullptr != p_begin && (*a_p_out_buffer_length) < config::command_line::callback_parameters_buffer_capacity)
    {
        assert(&(a_p_line[a_line_length]) > p_begin);
        a_p_out_buffer[(*a_p_out_buffer_length)++] = { p_begin,
                                                       static_cast<uint32_t>(&(a_p_line[a_line_length]) - p_begin) };
    }
}

bool Command_line::execute_command(const Callback::Parameter* a_p_parameters, uint32_t a_count)
{
    uint32_t index = config::command_line::callbacks_buffer_capacity;

    for (uint32_t i = 0;
         i < this->callback_buffer_length && config::command_line::callback_parameters_buffer_capacity != index;
         i++)
    {
        if (true ==
            cstring::equals(a_p_parameters[0].a_p_value, this->callbacks_buffer[i].p_name, a_p_parameters[0].length))
        {
            index = i;
        }
    }

    bool ret = index != config::command_line::callbacks_buffer_capacity;

    if (true == ret)
    {
        this->callbacks_buffer[index].function(a_p_parameters, a_count, this->callbacks_buffer[index].p_user_data);
    }

    return ret;
}

void Command_line::execute_escape_sequence(char a_first, char a_second)
{
    if ('[' == a_first && this->commands_carousel.get_length() > 0)
    {
        this->write_string.function("\033[2K\r", 5, this->write_string.p_user_data);
        this->write_prompt();

        switch (a_second)
        {
            case 'A': {
                const Commands_carousel::Command& command = this->commands_carousel.read_next();

                memory::copy(this->line_buffer, sizeof(this->line_buffer), command.buffer, command.length);
                this->line_length = command.length;

                this->write_string.function(command.buffer, command.length, this->write_string.p_user_data);
            }
            break;

            case 'B': {
                const Commands_carousel::Command& command = this->commands_carousel.read_prev();

                memory::copy(this->line_buffer, sizeof(this->line_buffer), command.buffer, command.length);
                this->line_length = command.length;

                this->write_string.function(command.buffer, command.length, this->write_string.p_user_data);
            }
            break;
        }
    }
}

void Command_line::Commands_carousel::push(const char* a_p_line, uint32_t a_length)
{
    if (this->write_index == config::command_line::commands_carousel_capacity)
    {
        this->write_index = 0;
    }

    this->commands[this->write_index].length = a_length;

    memory::copy(this->commands[this->write_index].buffer,
                 sizeof(this->commands[this->write_index].buffer),

                 a_p_line,
                 a_length);

    this->write_index++;

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