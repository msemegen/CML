/*
    Name: command_line.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <utils/command_line.hpp>

//cml
#include <common/assert.hpp>
#include <hal/usart.hpp>

namespace cml {
namespace utils {

using namespace common;
using namespace hal;

//void command_line::start()
//{
//    this->p_io_stream->read_bytes_it(io_stream_new_byte_callback, this);
//}
//
//void command_line::stop()
//{
//    this->p_io_stream->read_bytes_it(nullptr, nullptr);
//}
//
//void command_line::write_prompt()
//{
//}
//
//bool command_line::update()
//{
//    return false;
//}
//
//bool io_stream_new_byte_callback(byte a_byte, void* a_p_user_data)
//{
//    _assert(nullptr != a_p_user_data);
//
//    command_line* p_this = static_cast<command_line*>(a_p_user_data);
//    p_this->input_buffer.push_back(a_byte);
//
//    return true;
//}

} // namespace utils
} // namespace cml