/*
    Name: assert.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <cml/debug/assert.hpp>

namespace {

using namespace cml::debug;

assert::Halt halt { nullptr, nullptr };
assert::Print print { nullptr, nullptr };

} // namespace

namespace cml {
namespace debug {

void assert::register_halt(const Halt& a_callback)
{
    halt = a_callback;
}

void assert::register_print(const Print& a_callback)
{
    print = a_callback;
}

void assert::trap(const char* a_p_file, uint32_t a_line, const char* a_p_expression)
{
    if (nullptr != print.p_function)
    {
        print.p_function(print.p_user_data, a_p_file, a_line, a_p_expression);
    }

    if (nullptr != halt.p_function)
    {
        halt.p_function(halt.p_user_data);
    }
}

} // namespace debug
} // namespace cml