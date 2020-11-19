#pragma once

/*
    Name: cstring.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string_view>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assert.hpp>

namespace cml {

struct cstring
{
    static constexpr uint32_t format_number_buffer_capacity = 12;

    enum class Radix
    {
        bin = 2,
        oct = 8,
        dec = 10,
        hex = 16
    };

    static uint32_t length(const char* a_p_string, uint32_t a_max_length = std::numeric_limits<uint32_t>::max());
    static bool equals(const char* a_p_string_1, const char* a_p_string_2, uint32_t a_max_length);

    static void reverse(char* a_p_string, uint32_t a_length)
    {
        assert(nullptr != a_p_string);
        assert(a_length > 0);

        for (uint32_t i = 0; i < a_length / 2; i++)
        {
            std::swap((a_p_string[i]), (a_p_string[a_length - i - 1]));
        }
    }

    static uint32_t
    join(char* a_p_destination, uint32_t a_destination_capacity, const char* a_p_source, uint32_t a_source_length);

    template<typename Type_t> static Type_t to_signed_integer(const char* a_p_string, uint32_t a_length)
    {
        assert(nullptr != a_p_string && 0 != a_p_string[0]);
        assert(a_length > 0);

        Type_t retval = 0;

        for (uint32_t i = '-' == a_p_string[0] ? 1 : 0; i < a_length; i++)
        {
            assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
            retval = retval * 10u + (a_p_string[i] - '0');
        }

        return retval * ('-' == a_p_string[0] ? -1 : 1);
    }

    template<typename Type_t> static Type_t to_unsigned_integer(const char* a_p_string, uint32_t a_length)
    {
        assert(nullptr != a_p_string && 0 != a_p_string[0]);
        assert(a_length > 0);

        Type_t retval = 0;

        for (uint32_t i = 0; i < a_length; i++)
        {
            assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
            retval = retval * 10u + (a_p_string[i] - '0');
        }

        return retval;
    }

#ifdef CML_USE_FLOATING_POINT
    static float to_float(const char* a_p_string, uint32_t a_length)
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
#endif // CML_USE_FLOATING_POINT

    template<typename Type_t>
    static uint32_t from_unsigned_integer(Type_t a_value, char* a_p_buffer, uint32_t a_buffer_capacity, Radix a_base)
    {
        static_assert(false == std::numeric_limits<Type_t>::is_signed);

        assert(nullptr != a_p_buffer);
        assert(a_buffer_capacity > 1);

        uint32_t ret = 0u;

        if (0u == a_value)
        {
            a_p_buffer[0] = '0';
            a_p_buffer[1] = 0;

            return 1;
        }

        while (0 != a_value)
        {
            Type_t remainder  = a_value % static_cast<Type_t>(a_base);
            auto tt           = (remainder > 9u) ? (remainder - 10u) + 'A' : remainder + '0';
            a_p_buffer[ret++] = tt;
            a_value /= static_cast<Type_t>(a_base);
        }

        a_p_buffer[ret] = 0;
        reverse(a_p_buffer, ret);

        return ret;
    }

    template<typename Type_t>
    static uint32_t from_signed_integer(Type_t a_value, char* a_p_buffer, uint32_t a_buffer_capacity, Radix a_base)
    {
        static_assert(true == std::numeric_limits<Type_t>::is_signed);

        assert(nullptr != a_p_buffer);
        assert(a_buffer_capacity > 1);

        uint32_t ret  = 0;
        bool negative = a_value < 0;

        if (0 == a_value)
        {
            a_p_buffer[0] = '0';
            a_p_buffer[1] = 0;

            return 1;
        }

        while (0 != a_value)
        {
            Type_t remainder  = std::abs(a_value % static_cast<Type_t>(a_base));
            a_p_buffer[ret++] = (remainder > 9) ? (remainder - 10) + 'a' : remainder + '0';
            a_value /= static_cast<Type_t>(a_base);
        }

        if (true == negative)
        {
            a_p_buffer[ret++] = '-';
        }

        a_p_buffer[ret] = 0;
        reverse(a_p_buffer, ret);

        return ret;
    }

#ifdef CML_USE_FLOATING_POINT
    static uint32_t from_float(float a_value, char* a_p_buffer, uint32_t a_buffer_capacity, uint32_t a_afterpoint)
    {
        assert(nullptr != a_p_buffer);
        assert(a_buffer_capacity > 0);

        auto pow = [](int32_t x, uint32_t y) {
            int32_t ret = 1;

            while (y > 0)
            {
                if (true == is_bit_on(y, 0))
                {
                    ret = ret * x;
                }

                y = y >> 0x1u;
                x = x * x;
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
            a_p_buffer[l++] = '.';
            uint32_t d      = from_unsigned_integer(static_cast<uint32_t>(absf(a_value - i) * pow(10, a_afterpoint)),
                                               a_p_buffer + l,
                                               a_buffer_capacity - l,
                                               Radix::dec);

            if (d < a_afterpoint)
            {
                for (;d < a_afterpoint && l + d < a_buffer_capacity; d++)
                {
                    a_p_buffer[l + d] = '0';
                }

                a_p_buffer[l + d] = 0;
            }

            l += d;
        }

        return l;
    }
#endif // CML_USE_FLOATING_POINT

    template<typename... Types_t>
    static uint32_t format(char* a_p_buffer, uint32_t a_buffer_capacity, const char* a_p_format, Types_t... a_params)
    {
        const Argument args[] = { Argument { a_params }... };
        char number_buffer_data[format_number_buffer_capacity];

        Buffer destination_buffer { a_p_buffer, a_buffer_capacity };
        Buffer number_buffer { number_buffer_data, sizeof(number_buffer_data) };

        return format_raw(&destination_buffer, &number_buffer, a_p_format, args, sizeof...(a_params));
    }

    cstring()               = delete;
    cstring(cstring&&)      = delete;
    cstring(const cstring&) = delete;
    ~cstring()              = delete;

    cstring& operator=(cstring&&) = delete;
    cstring& operator=(const cstring&) = delete;

private:
    struct Buffer
    {
        char* p_data            = nullptr;
        const uint32_t capacity = 0;
    };

    class Argument
    {
    public:
        Argument()  = default;
        ~Argument() = default;

        Argument& operator=(Argument&&) = delete;
        Argument& operator=(const Argument&) = delete;

        Argument(Argument&& a_other)      = default;
        Argument(const Argument& a_other) = default;

        static_assert(sizeof(unsigned int) == sizeof(uint32_t));
        explicit Argument(unsigned int a_value)
            : data(a_value)
            , type(Type::unsigned_int_32)
        {
        }

        static_assert(sizeof(signed int) == sizeof(int32_t));
        explicit Argument(signed int a_value)
            : type(Type::signed_int_32)
        {
            memcpy(&(this->data), &(a_value), sizeof(a_value));
        }

        static_assert(sizeof(unsigned long int) == sizeof(uint32_t));
        explicit Argument(unsigned long int a_value)
            : Argument(static_cast<unsigned int>(a_value))
        {
        }

        static_assert(sizeof(signed long int) == sizeof(int32_t));
        explicit Argument(signed long int a_value)
            : Argument(static_cast<signed int>(a_value))
        {
        }

        static_assert(sizeof(unsigned short int) == sizeof(uint16_t));
        explicit Argument(unsigned short int a_value)
            : data(a_value)
            , type(Type::unsigned_int_16)
        {
        }

        static_assert(sizeof(signed short int) == sizeof(int16_t));
        explicit Argument(signed short int a_value)
            : type(Type::signed_int_16)
        {
            memcpy(&(this->data), &(a_value), sizeof(a_value));
        }

        static_assert(sizeof(unsigned char) == sizeof(uint8_t));
        explicit Argument(unsigned char a_value)
            : data(a_value)
            , type(Argument::Type::unsigned_int_32)
        {
        }

        static_assert(sizeof(signed char) == sizeof(int8_t));
        explicit Argument(signed char a_value)
            : type(Argument::Type::character)
        {
            memcpy(&(this->data), &(a_value), sizeof(a_value));
        }

        explicit Argument(const char* a_p_value)
            : type(Type::cstring)
        {
            memcpy(&(this->data), &(a_p_value), sizeof(a_p_value));
        }

        explicit Argument(float a_value)
            : type(Type::real)
        {
            memcpy(&(this->data), &(a_value), sizeof(a_value));
        }

        uint32_t get_uint32() const
        {
            assert(this->type == Type::unsigned_int_32 || this->type == Type::unsigned_int_16 ||
                   this->type == Type::unsigned_int_8);
            return this->data;
        }

        int32_t get_int32() const
        {
            assert(this->type == Type::signed_int_32 || this->type == Type::signed_int_16 ||
                   this->type == Type::signed_int_8);

            switch (this->type)
            {
                case Type::signed_int_8: {
                    int8_t v;
                    memcpy(&v, &(this->data), sizeof(v));
                    return v;
                }

                case Type::signed_int_16: {
                    int16_t v;
                    memcpy(&v, &(this->data), sizeof(v));
                    return v;
                }

                default: {
                    return static_cast<int32_t>(this->data);
                }
            }
        }

        float get_float() const
        {
            assert(Type::real == this->type);

            float v;
            memcpy(&v, &(this->data), sizeof(v));
            return v;
        }

        char get_char() const
        {
            assert(this->type == Type::character);
            return static_cast<char>(this->data);
        }

        const char* get_cstring() const
        {
            assert(this->type == Type::cstring);

            const char* p_v;
            memcpy(&p_v, &(this->data), sizeof(p_v));
            return p_v;
        }

    private:
        enum class Type
        {
            unsigned_int_8,
            unsigned_int_16,
            unsigned_int_32,
            signed_int_8,
            signed_int_16,
            signed_int_32,
            character,
            cstring,
            real,
            unknown,
        };

    private:
        uint32_t data = 0;
        Type type     = Type::unknown;
    };

private:
    static uint32_t format_raw(Buffer* a_p_destinaition_buffer,
                               Buffer* a_p_number_buffer,
                               const char* a_p_format,
                               const Argument* a_p_argv,
                               uint32_t a_argc);
};

} // namespace cml