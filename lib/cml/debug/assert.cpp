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
        char number_buffer[10] = { 0 };

        auto line_number_to_string = [&]() -> void {
            uint32_t i = 0;

            while (0 != a_line)
            {
                number_buffer[i++] = (a_line % 10u) + '0';
                a_line /= 10u;
            }

            for (uint32_t j = 0; j < i / 2; j++)
            {
                char temp                = number_buffer[j];
                number_buffer[j]         = number_buffer[i - j - 1];
                number_buffer[i - j - 1] = temp;
            }
        };

        line_number_to_string();
        print.p_function(a_p_file, number_buffer, a_p_expression, print.p_user_data);
    }

    if (nullptr != halt.p_function)
    {
        halt.p_function(halt.p_user_data);
    }
}

} // namespace debug
} // namespace cml