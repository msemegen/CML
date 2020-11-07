#pragma once

/*
    Name: cstring.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>
#include <cstdlib>
#include <limits>

// cml
#include <cml/common/memory.hpp>
#include <cml/debug/assert.hpp>

namespace cml {
namespace common {

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
            memory::swap(&(a_p_string[i]), &(a_p_string[a_length - i - 1]));
        }
    }

    static uint32_t
    join(char* a_p_destination, uint32_t a_destination_capacity, const char* a_p_source, uint32_t a_source_length);

    template<typename Type_t> static Type_t to_signed_integer(const char* a_p_string, uint32_t a_length)
    {
        assert(nullptr != a_p_string && 0 != a_p_string[0]);
        assert(a_length > 0);

        Type_t retval = 0;

        const Type_t min = '-' == a_p_string[0] ? 0 : 1;

        for (uint32_t i = a_length - 1, m = 1; i + a_length + min != a_length; i--, m *= 10)
        {
            assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
            retval += (a_p_string[i] - '0') * m;
        }

        return retval * ('-' == a_p_string[0] ? -1 : 1);
    }

    template<typename Type_t> static Type_t to_unsigned_integer(const char* a_p_string, uint32_t a_length)
    {
        assert(nullptr != a_p_string && 0 != a_p_string[0]);
        assert(a_length > 0);

        Type_t retval = 0;

        for (uint32_t i = a_length - 1, m = 1; i + a_length + 1 != a_length; i--, m *= 10)
        {
            retval += (a_p_string[i] - '0') * m;
        }

        return retval;
    }

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
            : data(*(reinterpret_cast<uint32_t*>(&a_value)))
            , type(Type::signed_int_32)
        {
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
            : data(*(reinterpret_cast<uint32_t*>(&a_value)))
            , type(Type::signed_int_16)
        {
        }

        static_assert(sizeof(unsigned char) == sizeof(uint8_t));
        explicit Argument(unsigned char a_value)
            : data(a_value)
            , type(Argument::Type::unsigned_int_32)
        {
        }

        static_assert(sizeof(signed char) == sizeof(int8_t));
        explicit Argument(signed char a_value)
            : data(*(reinterpret_cast<uint32_t*>(&a_value)))
            , type(Argument::Type::character)
        {
        }

        explicit Argument(const char* a_p_value)
            : data(*(reinterpret_cast<uint32_t*>(&a_p_value)))
            , type(Type::cstring)
        {
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
                    return static_cast<int32_t>(static_cast<int8_t>(this->data));
                }

                case Type::signed_int_16: {
                    return static_cast<int32_t>(static_cast<int16_t>(this->data));
                }

                default: {
                    return static_cast<int32_t>(this->data);
                }
            }
        }

        char get_char() const
        {
            assert(this->type == Type::character);
            return static_cast<char>(this->data);
        }

        const char* get_cstring() const
        {
            assert(this->type == Type::cstring);
            return reinterpret_cast<const char*>(this->data);
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

} // namespace common
} // namespace cml