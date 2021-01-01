/*
    Name: cstring.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <cml/cstring.hpp>

namespace cml {

using namespace cml::debug;

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
    assert(nullptr != a_p_destination);
    assert(a_destination_capacity > 0);

    assert(nullptr != a_p_source);
    assert(a_source_length > 0);

    uint32_t i = 0;
    for (; i < a_source_length && i + 1 < a_destination_capacity; i++)
    {
        a_p_destination[i] = a_p_source[i];
    }

    a_p_destination[i] = 0;

    return i;
}

#ifdef CML_USE_FLOATING_POINT
float cstring::to_float(const char* a_p_string, uint32_t a_length)
{
    assert(nullptr != a_p_string);
    assert(a_length > 0);

    float retval = 0.0f;
    float p      = 10.0f;
    uint32_t i   = '-' == a_p_string[0] ? 1 : 0;

    for (; i < a_length && '.' != a_p_string[i]; i++)
    {
        assert((a_p_string[i] >= '0' && a_p_string[i] <= '9') || '.' == a_p_string[i]);
        retval = retval * 10.0f + (a_p_string[i] - '0');
    }

    for (i = i + 1; i < a_length; i++, p *= 10.0f)
    {
        assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
        retval = retval + (a_p_string[i] - '0') / p;
    }

    return retval * ('-' == a_p_string[0] ? -1.0f : 1.0f);
}

uint32_t cstring::from_float(float a_value, char* a_p_buffer, uint32_t a_buffer_capacity, uint32_t a_afterpoint)
{
    assert(nullptr != a_p_buffer);
    assert(a_buffer_capacity > 0);

    auto pow = [](int32_t x, uint32_t y) {
        int32_t ret = 1;

        while (y > 0)
        {
            if (true == bit::is(y, 0))
            {
                ret = ret * x;
            }

            y = y >> 0x1u;
            x = x * x;
        }
        return ret;
    };

    auto get_digits_count = [](uint32_t a_value) -> auto
    {
        uint32_t ret = 1;

        if (a_value > 0)
        {
            for (ret = 0; a_value != 0; a_value /= 10, ret++)
                ;
        }

        return ret;
    };

    auto absf = [](float a_v) { return a_v > 0 ? a_v : -1 * a_v; };

    uint32_t offset = 0;
    int32_t i       = static_cast<int32_t>(a_value);

    if (i == 0 && a_value < 0.0f)
    {
        a_p_buffer[0] = '-';
        offset        = 1;
    }

    uint32_t l = from_signed_integer(i, a_p_buffer + offset, a_buffer_capacity - offset, Radix::dec) + offset;

    if (a_afterpoint > 0)
    {
        a_p_buffer[l++]         = '.';
        uint32_t decimal        = static_cast<uint32_t>(absf(a_value - i) * pow(10, a_afterpoint));
        uint32_t decimal_length = get_digits_count(decimal);
        offset                  = a_afterpoint - decimal_length;

        if (offset > 0)
        {
            const uint32_t m = l + offset;
            for (; l < m; l++)
            {
                a_p_buffer[l] = '0';
            }
        }

        uint32_t decimal_cstring_length =
            from_unsigned_integer(decimal, a_p_buffer + l, a_buffer_capacity - l, Radix::dec);

        if (decimal_cstring_length + offset < a_afterpoint)
        {
            for (uint32_t i = decimal_cstring_length; i < a_afterpoint && l + i < a_buffer_capacity; i++)
            {
                a_p_buffer[l + i] = '0';
            }

            a_p_buffer[l + i] = 0;
        }

        l += decimal_cstring_length;
    }

    return l;
}
#endif // CML_USE_FLOATING_POINT

uint32_t cstring::format_raw(Buffer* a_p_destinaition_buffer,
                             Buffer* a_p_number_buffer,
                             const char* a_p_format,
                             const Argument* a_p_argv,
                             uint32_t a_argc)
{
    bool argument           = false;
    uint32_t argument_index = 0;
    uint32_t length         = 0;
    uint32_t number_length  = 0;

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

                case 'x': {
                    number_length = cstring::from_unsigned_integer(a_p_argv[argument_index++].get_uint32(),
                                                                   a_p_number_buffer->p_data,
                                                                   a_p_number_buffer->capacity,
                                                                   cstring::Radix::hex);

                    length += cstring::join(a_p_destinaition_buffer->p_data + length,
                                            a_p_destinaition_buffer->capacity - length,
                                            a_p_number_buffer->p_data,
                                            number_length);

                    argument = false;
                }
                break;

                case 'u': {
                    number_length = cstring::from_unsigned_integer(a_p_argv[argument_index++].get_uint32(),
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

                case 'f': {
#ifdef CML_USE_FLOATING_POINT
                    number_length = cstring::from_float(a_p_argv[argument_index++].get_float(),
                                                        a_p_number_buffer->p_data,
                                                        a_p_number_buffer->capacity,
                                                        6);

                    length += cstring::join(a_p_destinaition_buffer->p_data + length,
                                            a_p_destinaition_buffer->capacity - length,
                                            a_p_number_buffer->p_data,
                                            number_length);

                    argument = false;
#else
                    assert(false);
#endif // CML_USE_FLOATING_POINT
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

} // namespace cml