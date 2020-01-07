/*
    Name: format.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <common/format.hpp>

namespace cml {
namespace common {

using namespace collection;

void c_format::raw(c_string* a_p_out_string,
                   const char* a_p_format,
                   const c_argument* a_p_argv,
                   uint32 a_argc)
{
    bool out_string_full = a_p_out_string->is_full();
    bool argument = false;

    while (*a_p_format != '\0' && false == out_string_full)
    {
        if (true == argument)
        {
            switch (*a_p_format)
            {
                case 'd':
                case 'i':
                {

                }
                break;

                case 'u':
                {

                }
                break;

                case 'x':
                {

                }
                break;

                case 'X':
                {

                }
                break;

                case 'c':
                {

                }
                break;

                case 's':
                {

                }
                break;

                case 'p':
                {

                }
                break;
            }
        }
        else
        {
            argument = *a_p_format == '%';

            if (false == argument)
            {
                a_p_out_string->push_back(*a_p_format);
                out_string_full = a_p_out_string->is_full();
            }
        }

        a_p_out_string++;
    }
}

} // namespace common
} // namespace cml