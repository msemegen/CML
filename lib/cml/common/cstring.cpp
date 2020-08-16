/*
    Name: cstring.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <cml/common/cstring.hpp>

namespace cml {
namespace common {

uint32_t cstring::length(const char* a_p_string, uint32_t a_max_length)
{
    uint32_t i = 0;
    while ((*a_p_string) != '\0' && i < a_max_length)
    {
        i++;
        a_p_string++;
    }

    return i;
}

bool cstring::equals(const char* a_p_string_1, const char* a_p_string_2, uint32_t a_max_length)
{
    bool the_same = true;

    for (decltype(a_max_length) i = 0;

         i < a_max_length && true == (the_same = (*a_p_string_1) == *(a_p_string_2)) && 0 != (*a_p_string_1) &&
         0 != (*a_p_string_2);

         i++, a_p_string_1++, a_p_string_2++)
        ;

    return the_same;
}

uint32_t
cstring::join(char* a_p_destination, uint32_t a_destination_capacity, const char* a_p_source, uint32_t a_source_length)
{
    uint32_t i = 0;
    for (; i < a_source_length && i + 1 < a_destination_capacity; i++)
    {
        a_p_destination[i] = a_p_source[i];
    }

    a_p_destination[i] = 0;

    return i;
}

uint32_t cstring::format_raw(Buffer* a_p_destinaition_buffer,
                             Buffer* a_p_number_buffer,
                             const char* a_p_format,
                             const Argument* a_p_argv,
                             uint32_t a_argc)
{
    bool argument           = false;
    uint32_t argument_index = 0;
    uint32_t length         = 0;

    while (*a_p_format != '\0' && length + 1 < a_p_destinaition_buffer->capacity)
    {
        if (true == argument && argument_index < a_argc)
        {
            switch (*a_p_format)
            {
                case 'd':
                case 'i': {
                    uint32_t number_length = cstring::from_signed_integer(a_p_argv[argument_index++].get_int32(),
                                                                          a_p_number_buffer->p_data,
                                                                          a_p_number_buffer->capacity,
                                                                          cstring::Radix::dec);

                    length += cstring::join(a_p_destinaition_buffer->p_data + length,
                                            a_p_destinaition_buffer->capacity - length,
                                            a_p_number_buffer->p_data,
                                            number_length);

                    argument = false;
                }
                break;

                case 'u': {
                    uint32_t number_length = cstring::from_unsigned_integer(a_p_argv[argument_index++].get_uint32(),
                                                                            a_p_number_buffer->p_data,
                                                                            a_p_number_buffer->capacity,
                                                                            cstring::Radix::dec);

                    length += cstring::join(a_p_destinaition_buffer->p_data + length,
                                            a_p_destinaition_buffer->capacity - length,
                                            a_p_number_buffer->p_data,
                                            number_length);

                    argument = false;
                }
                break;

                case 'c': {
                    a_p_destinaition_buffer->p_data[length++] = a_p_argv[argument_index++].get_char();

                    argument = false;
                }
                break;

                case 's': {
                    length += cstring::join(a_p_destinaition_buffer->p_data + length,
                                            a_p_destinaition_buffer->capacity - length,
                                            a_p_argv[argument_index].get_cstring(),
                                            cstring::length(a_p_argv[argument_index].get_cstring(),
                                                            a_p_destinaition_buffer->capacity - length));

                    argument_index++;
                    argument = false;
                }
                break;

                case '%': {
                    a_p_destinaition_buffer->p_data[length++] = '%';
                    argument                                  = false;
                }
                break;
            }
        }
        else
        {
            argument = *a_p_format == '%';

            if (false == argument)
            {
                a_p_destinaition_buffer->p_data[length++] = *a_p_format;
            }
        }

        a_p_format++;
    }

    a_p_destinaition_buffer->p_data[length] = 0;

    return length;
}

} // namespace common
} // namespace cml