/*
    Name: Logger.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <cml/utils/Logger.hpp>

//cml
#include <cml/common/cstring.hpp>
#include <cml/debug/assert.hpp>

namespace {

using namespace cml;

constexpr const char* tags[] =
{
    "[inf] ",
    "[wrn] ",
    "[err] ",
    "[omg] "
};

constexpr uint32_t tag_length = 6;

} // namespace ::

namespace cml {
namespace utils {

using namespace cml::common;

void Logger::inf(const char* a_p_message)
{
    this->write(a_p_message, Stream_type::inf);
}

void Logger::wrn(const char* a_p_message)
{
    this->write(a_p_message, Stream_type::wrn);
}

void Logger::err(const char* a_p_message)
{
    this->write(a_p_message, Stream_type::err);
}

void Logger::omg(const char* a_p_message)
{
    this->write(a_p_message, Stream_type::omg);
}

void Logger::write(const char* a_p_message, Stream_type a_type)
{
    assert(nullptr != this->p_io_stream);

    if (true == this->is_stream_enabled(a_type))
    {
        const uint32_t message_length = cstring::length(a_p_message, config::console::line_buffer_capacity);

        this->p_io_stream->transmit_bytes_polling(tags[static_cast<uint32_t>(a_type)], tag_length);
        this->p_io_stream->transmit_bytes_polling(a_p_message, message_length);
        this->p_io_stream->transmit_bytes_polling(&(config::new_line_character), 1);
    }
}

} // namespace utils
} // namespace cmls