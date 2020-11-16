#pragma once

/*
    Name: Logger.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>
#include <cstring>
#include <utility>

// cml
#include <cml/bit.hpp>
#include <cml/cstring.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/utils/config.hpp>

namespace cml {
namespace utils {

class Logger
{
public:
    enum class Stream_type : uint8_t
    {
        inf,
        wrn,
        err,
        omg,
    };

    struct Write_string_handler
    {
        using Function = uint32_t (*)(const char* a_p_string, uint32_t a_length, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Logger()
        : settings(0)
    {
    }

    Logger(const Write_string_handler& a_write_string_handler,
           bool a_inf,
           bool a_wrn,
           bool a_err,
           bool a_omg,
           bool a_draw_label)
        : write_string(a_write_string_handler)
        , settings(0)
    {
        assert(nullptr != a_write_string_handler.function);

        this->set_settings(a_inf, a_wrn, a_err, a_omg, a_draw_label);
    }

    Logger(Logger&&)      = default;
    Logger(const Logger&) = default;
    ~Logger()             = default;

    Logger& operator=(Logger&&) = default;
    Logger& operator=(const Logger&) = default;

    void set_calm_down()
    {
        clear_flag(&(this->settings), 0xFu);
    }

    void set_verbose()
    {
        set_flag(&(this->settings), static_cast<uint8_t>(0xFu));
    }

    uint32_t inf(const char* a_p_message)
    {
        return this->write(Stream_type::inf, a_p_message);
    }

    uint32_t wrn(const char* a_p_message)
    {
        return this->write(Stream_type::wrn, a_p_message);
    }

    uint32_t err(const char* a_p_message)
    {
        return this->write(Stream_type::err, a_p_message);
    }

    uint32_t omg(const char* a_p_message)
    {
        return this->write(Stream_type::omg, a_p_message);
    }

    template<typename... params> uint32_t inf(const char* a_p_format, params... a_params)
    {
        return this->write(Stream_type::inf, a_p_format, a_params...);
    }

    template<typename... params> uint32_t wrn(const char* a_p_format, params... a_params)
    {
        return this->write(Stream_type::wrn, a_p_format, a_params...);
    }

    template<typename... params> uint32_t err(const char* a_p_format, params... a_params)
    {
        return this->write(Stream_type::err, a_p_format, a_params...);
    }

    template<typename... params> uint32_t omg(const char* a_p_format, params... a_params)
    {
        return this->write(Stream_type::omg, a_p_format, a_params...);
    }

    void set_streams(bool a_inf, bool a_wrn, bool a_err, bool a_omg)
    {
        this->set_settings(a_inf, a_wrn, a_err, a_omg, this->is_label());
    }

    void set_label(bool a_flag)
    {
        if (true == a_flag)
        {
            set_bit(&(this->settings), 4u);
        }
        else
        {
            clear_bit(&(this->settings), 4u);
        }
    }

    bool is_stream_enabled(Stream_type a_type) const
    {
        return is_bit_on(this->settings, static_cast<uint32_t>(a_type));
    }

    bool is_label() const
    {
        return is_bit_on(this->settings, 4u);
    }

private:
    static constexpr const char* tags[] = { "[inf] ", "[wrn] ", "[err] ", "[omg] " };

private:
    uint32_t write(Stream_type a_type, const char* a_p_message)
    {
        if (true == this->is_stream_enabled(a_type))
        {
            uint32_t offset = 0;

            if (true == this->is_label())
            {
                static_assert(config::logger::line_buffer_capacity > 6);

                memcpy(this->line_buffer, tags[static_cast<uint32_t>(a_type)], 6u);
                offset = 6u;
            }

            uint32_t msglen  = cstring::length(a_p_message, config::logger::line_buffer_capacity - offset);
            uint32_t buffcap = config::logger::line_buffer_capacity - offset;
            uint32_t length  = msglen > buffcap ? buffcap : msglen;

            memcpy(this->line_buffer + offset, a_p_message, length);

            return this->write_string.function(this->line_buffer, length + offset, this->write_string.p_user_data);
        }

        return 0;
    }

    template<typename... params> uint32_t write(Stream_type a_type, const char* a_p_format, params... a_params)
    {
        if (true == this->is_stream_enabled(a_type))
        {
            uint32_t offset = 0;

            if (true == this->is_label())
            {
                static_assert(config::logger::line_buffer_capacity > 6);

                memcpy(this->line_buffer, tags[static_cast<uint32_t>(a_type)], 6u);
                offset = 6u;
            }

            uint32_t length = cstring::format(
                this->line_buffer + offset, config::logger::line_buffer_capacity - offset, a_p_format, a_params...);

            return this->write_string.function(this->line_buffer, length + offset, this->write_string.p_user_data);
        }

        return 0;
    }

    void set_settings(bool a_inf, bool a_wrn, bool a_err, bool a_omg, bool a_label)
    {
        set_flag(&(this->settings),
                 0xFu,
                 (true == a_inf ? 0x1u : 0x0u) | (true == a_wrn ? 0x2u : 0x0u) | (true == a_err ? 0x4u : 0x0u) |
                     (true == a_omg ? 0x8u : 0x0u) | (true == a_label ? 0x10u : 0x0u));
    }

private:
    Write_string_handler write_string;
    uint8_t settings;

    char line_buffer[config::logger::line_buffer_capacity];
};

} // namespace utils
} // namespace cml