#pragma once

/*
    Name: Logger.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

//cml
#include <cml/bit.hpp>
#include <cml/common/cstring.hpp>
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
        omg
    };

    struct Write_string_handler
    {
        using Function = uint32_t(*)(const char* a_p_string, uint32_t a_length, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:

    Logger()
        : verbosity(0)
    {}

    Logger(const Write_string_handler& a_write_string_handler, bool a_inf, bool a_wrn, bool a_err, bool a_omg)
        : write_string(a_write_string_handler)
        , verbosity(0)
    {
        assert(nullptr != a_write_string_handler.function);

        this->set_verbosity(a_inf, a_wrn, a_err, a_omg);
    }

    Logger(Logger&&)      = default;
    Logger(const Logger&) = default;
    ~Logger()             = default;

    Logger& operator = (Logger&&)      = default;
    Logger& operator = (const Logger&) = default;

    bool is_stream_enabled(Stream_type a_type)
    {
        return is_bit(this->verbosity, static_cast<uint32_t>(a_type));
    }

    void set_verbosity(bool a_inf, bool a_wrn, bool a_err, bool a_omg)
    {
        set_flag(&(this->verbosity), 0xFu, this->create_verbosity_mask(a_inf, a_wrn, a_err, a_omg));
    }

    void set_calm_down()
    {
        this->verbosity = 0;
    }

    void set_verbose()
    {
        set_flag(&(this->verbosity), static_cast<uint8_t>(0xFu));
    }

    uint32_t inf(const char* a_p_message)
    {
        return this->write(a_p_message, Stream_type::inf);
    }

    uint32_t wrn(const char* a_p_message)
    {
        return this->write(a_p_message, Stream_type::wrn);
    }

    uint32_t err(const char* a_p_message)
    {
        return this->write(a_p_message, Stream_type::err);
    }

    uint32_t omg(const char* a_p_message)
    {
        return this->write(a_p_message, Stream_type::omg);
    }

    template<typename ... params>
    uint32_t inf(const char* a_p_format, params ... a_params)
    {
        if (true == this->is_stream_enabled(Stream_type::inf))
        {
            common::memory::copy(this->line_buffer, config::logger::line_buffer_capacity, tags[0], 6);

            uint32_t length = common::cstring::format(this->line_buffer + 6,
                                                      config::logger::line_buffer_capacity - 6,
                                                      a_p_format,
                                                      a_params...);

            return this->write_string.function(this->line_buffer, length + 6, this->write_string.p_user_data);
        }

        return 0;
    }

    template<typename ... params>
    uint32_t wrn(const char* a_p_format, params ... a_params)
    {
        if (true == this->is_stream_enabled(Stream_type::wrn))
        {
            common::memory::copy(this->line_buffer, config::logger::line_buffer_capacity, tags[1], 6);

            uint32_t length = common::cstring::format(this->line_buffer + 6,
                                                      config::logger::line_buffer_capacity - 6,
                                                      a_p_format,
                                                      a_params...);

            return this->write_string.function(this->line_buffer, length + 6, this->write_string.p_user_data);
        }

        return 0;
    }

    template<typename ... params>
    uint32_t err(const char* a_p_format, params ... a_params)
    {
        if (true == this->is_stream_enabled(Stream_type::err))
        {
           common::memory::copy(this->line_buffer, config::logger::line_buffer_capacity, tags[2], 6);

           uint32_t length = common::cstring::format(this->line_buffer + 6,
                                                     config::logger::line_buffer_capacity - 6,
                                                     a_p_format,
                                                     a_params...);

           return this->write_string.function(this->line_buffer, length + 6, this->write_string.p_user_data);
        }

        return 0;
    }

    template<typename ... params>
    uint32_t omg(const char* a_p_format, params ... a_params)
    {
        if (true == this->is_stream_enabled(Stream_type::omg))
        {
            common::memory::copy(this->line_buffer, config::logger::line_buffer_capacity, tags[3], 6);

            uint32_t length = common::cstring::format(this->line_buffer + 6,
                                                      config::logger::line_buffer_capacity - 6,
                                                      a_p_format,
                                                      a_params...);

            return this->write_string.function(this->line_buffer, length + 6, this->write_string.p_user_data);
        }

        return 0;
    }

private:

    static constexpr const char* tags[] =
    {
        "[inf] ",
        "[wrn] ",
        "[err] ",
        "[omg] "
    };

private:

    uint32_t write(const char* a_p_message, Stream_type a_type)
    {
        common::memory::copy(this->line_buffer,
                             config::logger::line_buffer_capacity,
                             tags[static_cast<uint32_t>(a_type)],
                             6);

        uint32_t length = common::memory::copy(this->line_buffer + 6,
                                               config::logger::line_buffer_capacity - 6,
                                               a_p_message,
                                               common::cstring::length(a_p_message,
                                                                       config::logger::line_buffer_capacity - 6));

        return this->write_string.function(this->line_buffer, length + 6, this->write_string.p_user_data);
    }

    uint8_t create_verbosity_mask(bool a_inf, bool a_wrn, bool a_err, bool a_omg)
    {
        return (true == a_inf ? 0x1u : 0x0u) |
               (true == a_wrn ? 0x2u : 0x0u) |
               (true == a_err ? 0x4u : 0x0u) |
               (true == a_omg ? 0x8u : 0x0u);
    }

private:

    Write_string_handler write_string;
    uint8_t verbosity;

    char line_buffer[config::logger::line_buffer_capacity];
};

} // namepace hal
} // namepace cml