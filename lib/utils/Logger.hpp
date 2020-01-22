#pragma once

/*
    Name: Logger.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <collection/string.hpp>
#include <common/bit.hpp>
#include <common/format.hpp>
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

    void enable(hal::USART* a_p_err_stream, bool a_inf, bool a_wrn, bool a_err, bool a_omg)
    {
        this->p_err_stream = a_p_err_stream;
        this->set_verbosity(a_inf, a_wrn, a_err, a_omg);
    }

    void disable()
    {
        this->p_err_stream = nullptr;
    }

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

    void calm_down()
    {
        this->verbosity = 0;
    }

    void verbose()
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
        this->line_buffer_view.clear();
        common::format(&(this->line_buffer_view), a_p_format, a_params...);
        this->inf(this->line_buffer);
    }

    template<typename ... params>
    void wrn(const char* a_p_format, params ... a_params)
    {
        this->line_buffer_view.clear();
        common::format(&(this->line_buffer_view), a_p_format, a_params...);
        this->wrn(this->line_buffer);
    }

    template<typename ... params>
    void err(const char* a_p_format, params ... a_params)
    {
        this->line_buffer_view.clear();
        common::format(&(this->line_buffer_view), a_p_format, a_params...);
        this->err(this->line_buffer);
    }

    template<typename ... params>
    void omg(const char* a_p_format, params ... a_params)
    {
        this->line_buffer_view.clear();
        common::format(&(this->line_buffer_view), a_p_format, a_params...);
        this->omg(this->line_buffer);
    }

private:

    Logger()              = default;
    Logger(Logger&&)      = delete;
    Logger(const Logger&) = delete;

    Logger& operator = (const Logger&) = delete;
    Logger& operator = (Logger&&)      = delete;

    void write(const char* a_p_message, Stream_type a_type);

private:

    hal::USART* p_err_stream;
    common::uint8 verbosity;

    char line_buffer[config::console::line_buffer_capacity];
    collection::String line_buffer_view;
};

} // namepace hal
} // namepace cml