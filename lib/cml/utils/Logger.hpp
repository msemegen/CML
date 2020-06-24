#pragma once

/*
    Name: Logger.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/bit.hpp>
#include <cml/integer.hpp>
#include <cml/common/cstring.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/utils/config.hpp>

namespace cml {
namespace utils {

class Logger
{
public:

    enum class Stream_type : uint8
    {
        inf,
        wrn,
        err,
        omg
    };

public:

    Logger()
        : p_io_stream(nullptr)
        , verbosity(0)
    {}

    Logger(hal::peripherals::USART* a_p_stream, bool a_inf, bool a_wrn, bool a_err, bool a_omg)
        : p_io_stream(a_p_stream)
        , verbosity(0)
    {
        this->set_verbosity(a_inf, a_wrn, a_err, a_omg);
    }

    Logger(Logger&&)      = default;
    Logger(const Logger&) = default;
    ~Logger()             = default;

    Logger& operator = (Logger&&)      = default;
    Logger& operator = (const Logger&) = default;

    bool is_stream_enabled(Stream_type a_type)
    {
        return is_bit(this->verbosity, static_cast<uint32>(a_type));
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
        set_flag(&(this->verbosity), static_cast<uint8>(0xFu));
    }

    void inf(const char* a_p_message);
    void wrn(const char* a_p_message);
    void err(const char* a_p_message);
    void omg(const char* a_p_message);

    template<typename ... params>
    void inf(const char* a_p_format, params ... a_params)
    {
        common::cstring::format(this->line_buffer, config::logger::line_buffer_capacity, a_p_format, a_params...);
        this->inf(this->line_buffer);
    }

    template<typename ... params>
    void wrn(const char* a_p_format, params ... a_params)
    {
        common::cstring::format(this->line_buffer, config::logger::line_buffer_capacity, a_p_format, a_params...);
        this->wrn(this->line_buffer);
    }

    template<typename ... params>
    void err(const char* a_p_format, params ... a_params)
    {
        common::cstring::format(this->line_buffer, config::logger::line_buffer_capacity, a_p_format, a_params...);
        this->err(this->line_buffer);
    }

    template<typename ... params>
    void omg(const char* a_p_format, params ... a_params)
    {
        common::cstring::format(this->line_buffer, config::logger::line_buffer_capacity, a_p_format, a_params...);
        this->omg(this->line_buffer);
    }

private:

    void write(const char* a_p_message, Stream_type a_type);

    uint8 create_verbosity_mask(bool a_inf, bool a_wrn, bool a_err, bool a_omg)
    {
        return (true == a_inf ? 0x1u : 0x0u) | 
               (true == a_wrn ? 0x2u : 0x0u) |
               (true == a_err ? 0x4u : 0x0u) |
               (true == a_omg ? 0x8u : 0x0u);
    }

private:

    hal::peripherals::USART* p_io_stream;
    uint8 verbosity;

    char line_buffer[config::logger::line_buffer_capacity];
};

} // namepace hal
} // namepace cml