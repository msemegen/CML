#pragma once

/*
    Name: Logger.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/bit.hpp>
#include <common/cstring.hpp>
#include <common/integer.hpp>
#include <hal/usart.hpp>
#include <utils/config.hpp>

namespace cml {
namespace utils {

class Logger
{
public:

    enum class Stream_type : common::uint8
    {
        inf,
        wrn,
        err,
        omg
    };

public:

    Logger()
        : p_io_stream(nullptr)
    {}

    Logger(hal::USART* a_p_stream)
        : p_io_stream(a_p_stream)
    {}

    Logger(Logger&&)      = default;
    Logger(const Logger&) = default;
    ~Logger()             = default;

    Logger& operator = (Logger&&)      = default;
    Logger& operator = (const Logger&) = default;

    bool is_stream_enabled(Stream_type a_type)
    {
        return common::get_bit(this->verbosity, static_cast<common::uint32>(a_type));
    }

    void set_verbosity(bool a_inf, bool a_wrn, bool a_err, bool a_omg)
    {
        common::clear_bit(&(this->verbosity), static_cast<common::int8>(Stream_type::inf));
        common::clear_bit(&(this->verbosity), static_cast<common::int8>(Stream_type::wrn));
        common::clear_bit(&(this->verbosity), static_cast<common::int8>(Stream_type::err));
        common::clear_bit(&(this->verbosity), static_cast<common::int8>(Stream_type::omg));

        if (true == a_inf) { common::set_bit(&(this->verbosity), static_cast<common::uint8>(Stream_type::inf)); }
        if (true == a_wrn) { common::set_bit(&(this->verbosity), static_cast<common::uint8>(Stream_type::wrn)); }
        if (true == a_err) { common::set_bit(&(this->verbosity), static_cast<common::uint8>(Stream_type::err)); }
        if (true == a_omg) { common::set_bit(&(this->verbosity), static_cast<common::uint8>(Stream_type::omg)); }
    }

    void set_calm_down()
    {
        this->verbosity = 0;
    }

    void set_verbose()
    {
        common::set_flag(&(this->verbosity), static_cast<common::uint8>(0xFu));
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

private:

    hal::USART* p_io_stream;
    common::uint8 verbosity;

    char line_buffer[config::logger::line_buffer_capacity];
};

} // namepace hal
} // namepace cml