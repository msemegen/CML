#pragma once

/*
    Name: Command_line.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// cml
#include <cml/cstring.hpp>
#include <cml/debug/assert.hpp>
#include <cml/utils/config.hpp>

namespace cml {
namespace utils {

class Command_line
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

    struct Callback
    {
        struct Parameter
        {
            const char* a_p_value = nullptr;
            uint32_t length       = 0;
        };

        using Function = void (*)(const Parameter*, uint32_t, void*);

        const char* p_name = nullptr;

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Command_line(const Write_character_handler& a_write_character_handler,
                 const Write_string_handler& a_write_string_handler,
                 const Read_character_handler& a_read_character_handler,
                 const char* a_p_prompt,
                 const char* a_p_command_not_found_message)
        : write_character(a_write_character_handler)
        , write_string(a_write_string_handler)
        , read_character(a_read_character_handler)
        , p_prompt(a_p_prompt)
        , p_command_not_found_message(a_p_command_not_found_message)
        , prompt_length(cstring::length(a_p_prompt, config::command_line::line_buffer_capacity))
        , command_not_found_message_length(
              cstring::length(a_p_command_not_found_message, config::command_line::line_buffer_capacity))
        , line_length(0)
        , callback_parameters_buffer_length(0)
        , callback_buffer_length(0)
    {
        assert(nullptr != a_write_character_handler.function);
        assert(nullptr != a_write_string_handler.function);
        assert(nullptr != a_read_character_handler.function);
        assert(nullptr != a_p_prompt);
        assert(nullptr != p_command_not_found_message);
    }

    Command_line()                    = delete;
    Command_line(Command_line&&)      = default;
    Command_line(const Command_line&) = default;
    ~Command_line()                   = default;

    Command_line& operator=(Command_line&&) = default;
    Command_line& operator=(const Command_line&) = default;

    void update();

    void register_callback(const Callback& a_callback)
    {
        assert(this->callback_buffer_length < config::command_line::callbacks_buffer_capacity);
        this->callbacks_buffer[this->callback_buffer_length++] = a_callback;
    }

    void write_prompt()
    {
        this->write_string.function(this->p_prompt, this->prompt_length, this->write_string.p_user_data);
    }

private:
    class Commands_carousel
    {
    public:
        struct Command
        {
            char buffer[config::command_line::line_buffer_capacity] = { 0 };
            uint32_t length                                         = 0;
        };

    public:
        Commands_carousel()
            : length(0)
            , read_index(0)
            , write_index(0)
        {
        }

        Commands_carousel(Commands_carousel&&)      = default;
        Commands_carousel(const Commands_carousel&) = default;
        ~Commands_carousel()                        = default;

        Commands_carousel& operator=(Commands_carousel&&) = default;
        Commands_carousel& operator=(const Commands_carousel&) = default;

        void push(const char* a_p_line, uint32_t a_length);

        const Command& read_next() const;
        const Command& read_prev() const;

        uint32_t get_length() const
        {
            return this->length;
        }

    private:
        Command commands[config::command_line::commands_carousel_capacity];

        uint32_t length;
        mutable uint32_t read_index;
        mutable uint32_t write_index;
    };

private:
    void get_callback_parameters(Callback::Parameter* a_p_out_buffer,
                                 uint32_t a_out_buffor_capacity,
                                 uint32_t* a_p_out_buffer_length,
                                 char* a_p_line,
                                 uint32_t a_line_length,
                                 const char* a_p_separators,
                                 uint32_t a_separators_count);

    bool execute_command(const Callback::Parameter* a_p_parameters, uint32_t a_count);
    void execute_escape_sequence(char a_first, char a_second);

    void write_new_line()
    {
        this->write_character.function('\n', this->write_character.p_user_data);
    }

private:
    Write_character_handler write_character;
    Write_string_handler write_string;
    Read_character_handler read_character;

    const char* p_prompt;
    const char* p_command_not_found_message;

    const uint32_t prompt_length;
    const uint32_t command_not_found_message_length;
    uint32_t line_length;

    char line_buffer[config::command_line::line_buffer_capacity];

    Callback::Parameter callback_parameters_buffer[config::command_line::callback_parameters_buffer_capacity];
    Callback callbacks_buffer[config::command_line::callbacks_buffer_capacity];

    uint32_t callback_parameters_buffer_length;
    uint32_t callback_buffer_length;

    Commands_carousel commands_carousel;
};

} // namespace utils
} // namespace cml