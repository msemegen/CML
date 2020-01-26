//this
#include <common/cstring.hpp>

namespace cml {
namespace common {

uint32 cstring::length(const char* a_p_string, uint32 a_max_length)
{
    uint32 i = 0;
    while ((*a_p_string) != '\0' && i < a_max_length) { i++; a_p_string++; }

    return i;
}

bool cstring::the_same(const char* a_p_string_1, const char* a_p_string_2, uint32 a_max_length)
{
    bool the_same = true;

    for (decltype(a_max_length) i = 0;

         i < a_max_length &&
         true == (the_same = (*a_p_string_1) == *(a_p_string_2)) &&
         0 != (*a_p_string_1) &&
         0 != (*a_p_string_2);

         i++,
         a_p_string_1++,
         a_p_string_2++);

    return the_same;
}

uint32 cstring::join(char* a_p_destination,
                    uint32 a_destination_capacity,
                    const char* a_p_source,
                    uint32 a_source_length)
{
    uint32 i = 0;
    for (;i < a_source_length && i + 1 < a_destination_capacity; i++)
    {
        a_p_destination[i] = a_p_source[i];
    }

    a_p_destination[i] = 0;

    return i;
}

void cstring::format::raw(char* a_p_buffer,
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
                    uint32 number_length = cstring::from_dec_integer(a_p_argv[argument_index++].get_int32(),
                                                                     this->number_buffer,
                                                                     number_buffer_capacity);


                    length += cstring::join(a_p_buffer + length,
                                            a_buffer_capacity - length,
                                            this->number_buffer,
                                            number_length);

                    argument = false;
                }
                break;

                case 'u':
                {
                    uint32 number_length = cstring::from_dec_integer(a_p_argv[argument_index++].get_uint32(),
                                                                     this->number_buffer,
                                                                     number_buffer_capacity);


                    length += cstring::join(a_p_buffer + length,
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
                    length += cstring::join(a_p_buffer + length,
                                            a_buffer_capacity - length,
                                            a_p_argv[argument_index].get_cstring(),
                                            cstring::length(a_p_argv[argument_index].get_cstring(),
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