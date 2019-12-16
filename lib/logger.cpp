/*
    Name: logger.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <utils/logger.hpp>

//cml
#include <common/assert.hpp>
#include <common/cstring.hpp>
#include <common/integer.hpp>

namespace {

using namespace cml::common;

constexpr const char* tags[] =
{
    "[inf] ",
    "[wrn] ",
    "[err] ",
    "[omg] "
};

constexpr uint32 tag_length = 6;

} // namespace ::

namespace cml {
namespace utils {

using namespace cml::common;

void c_logger::inf(const char* a_p_message)
{
    this->write(a_p_message, e_stream_type::inf);
}

void c_logger::wrn(const char* a_p_message)
{
    this->write(a_p_message, e_stream_type::wrn);
}

void c_logger::err(const char* a_p_message)
{
    this->write(a_p_message, e_stream_type::err);
}

void c_logger::omg(const char* a_p_message)
{
    this->write(a_p_message, e_stream_type::omg);
}

void c_logger::write(const char* a_p_message, e_stream_type a_type)
{
    _assert(nullptr != this->p_err_stream);

    if (true == this->is_stream_enabled(a_type))
    {
        const uint32 message_length = cstring_length(a_p_message, s_config::s_console::line_buffer_capacity);

        this->p_err_stream->write_bytes_polling(tags[static_cast<uint32>(a_type)], tag_length);
        this->p_err_stream->write_bytes_polling(a_p_message, message_length);
        this->p_err_stream->write_bytes_polling(&(s_config::new_line_character), 1);
    }
}

} // namespace utils
} // namespace cml