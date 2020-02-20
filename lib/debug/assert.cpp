/*
    Name: assert.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <debug/assert.hpp>

namespace {

using namespace cml::debug;

assert::Halt_callback  halt{ nullptr, nullptr };
assert::Print_callback print{ nullptr, nullptr };

} // namespace ::

namespace cml {
namespace debug {

using namespace cml::common;

void assert::register_callback(const Halt_callback& a_callback)
{
    halt = a_callback;
}

void assert::register_callback(const Print_callback& a_callback)
{
    print = a_callback;
}

void assert::trap(const char* a_p_file, int32 a_line, const char* a_p_expression)
{
    if (nullptr != print.p_function)
    {
        print.p_function(print.p_user_data, a_p_file, a_line, a_p_expression);
    }

    if (nullptr != halt.p_function)
    {
        halt.p_function(halt.p_user_data);
    }

    while (true);
}

} // namespace debug
} // namespace cml