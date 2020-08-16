#pragma once

/*
    Name: Console.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// cml
#include <cml/common/cstring.hpp>
#include <cml/debug/assert.hpp>
#include <cml/utils/config.hpp>

namespace cml {
namespace utils {

class Console
{
public:
    struct Write_character_handler
    {
        using Function = uint32_t (*)(char a_character, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Write_string_handler
    {
        using Function = uint32_t (*)(const char* a_p_string, uint32_t a_length, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Read_character_handler
    {
        using Function = uint32_t (*)(char* a_p_out, uint32_t a_buffer_size, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Console(const Write_character_handler& a_write_character_handler,
            const Write_string_handler& a_write_string_handler,
            const Read_character_handler& a_read_character_handler)
        : write_character(a_write_character_handler)
        , write_string(a_write_string_handler)
        , read_character(a_read_character_handler)
    {
        assert(nullptr != a_write_character_handler.function);
        assert(nullptr != a_write_string_handler.function);
        assert(nullptr != a_read_character_handler.function);
    }

    Console()               = default;
    Console(Console&&)      = default;
    Console(const Console&) = default;
    ~Console()              = default;

    Console& operator=(Console&&) = default;
    Console& operator=(const Console&) = default;

    uint32_t write(char a_character)
    {
        return this->write_character.function(a_character, this->write_character.p_user_data);
    }

    uint32_t write(const char* a_p_string)
    {
        assert(nullptr != a_p_string);

        return this->write_string.function(a_p_string,
                                           common::cstring::length(a_p_string, config::console::line_buffer_capacity),
                                           this->write_string.p_user_data);
    }

    template<typename... Params_t> uint32_t write(const char* a_p_format, Params_t... a_params)
    {
        uint32_t length =
            common::cstring::format(this->line_buffer, config::console::line_buffer_capacity, a_p_format, a_params...);

        return this->write_string.function(this->line_buffer, length, this->write_string.p_user_data);
    }

    uint32_t write_line(char a_character)
    {
        const char str[] = { a_character, config::new_line_character };
        return this->write_string.function(str, 2, this->write_string.p_user_data);
    }

    uint32_t write_line(const char* a_p_string)
    {
        uint32_t length =
            common::memory::copy(this->line_buffer,
                                 config::command_line::line_buffer_capacity,

                                 a_p_string,
                                 common::cstring::length(a_p_string, config::command_line::line_buffer_capacity));

        if (length < config::command_line::line_buffer_capacity)
        {
            this->line_buffer[length++] = config::new_line_character;
        }
        else
        {
            this->line_buffer[length - 1] = config::new_line_character;
        }

        return this->write_string.function(this->line_buffer, length, this->write_string.p_user_data);
    }

    template<typename... Params_t> uint32_t write_line(const char* a_p_format, Params_t... a_params)
    {
        uint32_t length =
            common::cstring::format(this->line_buffer, config::console::line_buffer_capacity, a_p_format, a_params...);

        if (length < config::command_line::line_buffer_capacity)
        {
            this->line_buffer[length++] = config::new_line_character;
        }
        else
        {
            this->line_buffer[length - 1] = config::new_line_character;
        }

        return this->write_string.function(this->line_buffer, length, this->write_string.p_user_data);
    }

    uint32_t read_key(char* a_p_character)
    {
        assert(nullptr != a_p_character);

        return this->read_character.function(a_p_character, 1, this->read_character.p_user_data);
    }

    uint32_t read_line(char* a_p_buffer, uint32_t a_buffer_size)
    {
        assert(nullptr != a_p_buffer);
        assert(a_buffer_size > 0);

        uint32_t length = 0;

        do
        {
            length += this->read_key(a_p_buffer + length);

        } while (length < a_buffer_size && config::new_line_character != a_p_buffer[length - 1]);

        return length;
    }

private:
    Write_character_handler write_character;
    Write_string_handler write_string;
    Read_character_handler read_character;

    char line_buffer[config::console::line_buffer_capacity];
    char input_buffer[config::console::input_buffer_capacity];
};

} // namespace utils
} // namespace cml