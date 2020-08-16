#pragma once

/*
    Name: Command_line.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// cml
#include <cml/collection/Ring.hpp>
#include <cml/collection/Vector.hpp>
#include <cml/common/cstring.hpp>
#include <cml/debug/assert.hpp>
#include <cml/utils/config.hpp>

namespace cml {
namespace utils {

class Command_line
{
public:
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

        using Function = void (*)(const collection::Vector<Parameter>&, void*);

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
        , prompt_length(common::cstring::length(a_p_prompt, config::command_line::line_buffer_capacity))
        , command_not_found_message_length(
              common::cstring::length(a_p_command_not_found_message, config::command_line::line_buffer_capacity))
        , line_length(0)
        , callback_parameters_buffer_view(this->callback_parameters_buffer,
                                          config::command_line::callback_parameters_buffer_capacity)
        , callbacks_buffer_view(this->callbacks_buffer, config::command_line::callbacks_buffer_capacity)
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

    bool register_callback(const Callback& a_callback)
    {
        return this->callbacks_buffer_view.push_back(a_callback);
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
    collection::Vector<Callback::Parameter>
    get_callback_parameters(char* a_p_line, uint32_t a_length, const char* a_p_separators, uint32_t a_separators_count);

    bool execute_command(const collection::Vector<Callback::Parameter>& a_parameters);
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

    uint32_t prompt_length;
    uint32_t command_not_found_message_length;
    uint32_t line_length;

    char line_buffer[config::command_line::line_buffer_capacity];

    Callback::Parameter callback_parameters_buffer[config::command_line::callback_parameters_buffer_capacity];
    Callback callbacks_buffer[config::command_line::callbacks_buffer_capacity];

    collection::Vector<Callback::Parameter> callback_parameters_buffer_view;
    collection::Vector<Callback> callbacks_buffer_view;

    Commands_carousel commands_carousel;
};

} // namespace utils
} // namespace cml