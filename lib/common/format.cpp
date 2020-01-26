/*
    Name: format.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <common/cstring.hpp>
#include <common/format.hpp>

namespace cml {
namespace common {

void format::raw(char* a_p_buffer,
                 uint32 a_buffer_capacity,
                 const char* a_p_format,
                 const Argument* a_p_argv,
                 uint32 a_argc)
{
    bool argument          = false;
    uint32 argument_index  = 0;
    uint32 length          = 0;

    while (*a_p_format != '\0' && length + 1 < a_buffer_capacity)
    {
        if (true == argument && argument_index < a_argc)
        {
            switch (*a_p_format)
            {
                case 'd':
                case 'i':
                {
                    uint32 number_length = cstring_from_dec_integer(a_p_argv[argument_index++].get_int32(),
                                                                    this->number_buffer,
                                                                    number_buffer_capacity);


                    length += cstring_join(a_p_buffer + length,
                                           a_buffer_capacity - length,
                                           this->number_buffer,
                                           number_length);

                    argument = false;
                }
                break;

                case 'u':
                {
                    uint32 number_length = cstring_from_dec_integer(a_p_argv[argument_index++].get_uint32(),
                                                                    this->number_buffer,
                                                                    number_buffer_capacity);


                    length += cstring_join(a_p_buffer + length,
                                           a_buffer_capacity - length,
                                           this->number_buffer,
                                           number_length);

                    argument = false;
                }
                break;

                case 'c':
                {
                    a_p_buffer[length++] = a_p_argv[argument_index++].get_char();

                    argument = false;
                }
                break;

                case 's':
                {
                    length += cstring_join(a_p_buffer + length,
                                           a_buffer_capacity - length,
                                           a_p_argv[argument_index].get_cstring(),
                                           cstring_length(a_p_argv[argument_index].get_cstring(),
                                           a_buffer_capacity - length));

                    argument_index++;
                    argument = false;
                }
                break;

                case '%':
                {
                    a_p_buffer[length++] = '%';
                    argument = false;
                }
                break;
            }
        }
        else
        {
            argument = *a_p_format == '%';

            if (false == argument)
            {
                a_p_buffer[length++] = *a_p_format;
            }
        }

        a_p_format++;
    }

    a_p_buffer[length] = 0;
}

} // namespace common
} // namespace cml