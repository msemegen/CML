#pragma once

/*
    Name: cstring.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>
#include <common/memory.hpp>
#include <debug/assert.hpp>


namespace cml {
namespace common {

struct cstring
{
    enum class Radix
    {
        bin = 2,
        oct = 8,
        dec = 10,
        hex = 16
    };

    static uint32 length(const char* a_p_string, uint32 a_max_length);
    static bool equals(const char* a_p_string_1, const char* a_p_string_2, uint32 a_max_length);

    static uint32 join(char* a_p_destination,
                       uint32 a_destination_capacity,
                       const char* a_p_source,
                       uint32 a_source_length);

    template<typename Type_t>
    static Type_t to_integer(const char* a_p_string, uint32 a_length)
    {
        assert(nullptr != a_p_string && 0 != a_p_string[0]);
        assert(a_length > 0);

        Type_t retval = 0;

        const Type_t min = '-' == a_p_string[0] ? 0 : 1;

        for (uint32 i = a_length - 1, m = 1; i + a_length + min != a_length; i--, m *= 10)
        {
            assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
            retval += (a_p_string[i] - '0') * m;
        }

        return retval * ('-' == a_p_string[0] ? -1 : 1);
    }

    template<typename Type_t>
    static uint32 from_integer(Type_t a_value, char* a_p_buffer, uint32 a_buffer_capacity, Radix a_base)
    {
        assert(a_buffer_capacity > 1);

        uint32 length      = a_value > 0 ? 0 : 1;
        const uint32 start = a_value > 0 ? 0 : 1;

        a_p_buffer[start] = 0;

        if (a_value < 0)
        {
            a_value *= -1;
        }

        while (0 != a_value && length < a_buffer_capacity)
        {
            char v = static_cast<char>(a_value % static_cast<byte>(a_base));
            char to_insert = (v > 9) ? (v - 10) + 'a' : v + '0';

            memory::move(a_p_buffer + 1, a_p_buffer, ++length);
            a_p_buffer[start] = to_insert;

            a_value /= static_cast<Type_t>(a_base);
        }

        if (1 == start)
        {
            a_p_buffer[0] = '-';
        }

        return length;
    }

    template<typename ... Types_t>
    static uint32 format(char* a_p_buffer, uint32 a_buffer_capacity, const char* a_p_format, Types_t ... a_params)
    {
        const Argument args[] = { Argument{a_params}... };
        char number_buffer_data[22];

        Buffer destination_buffer{ a_p_buffer, a_buffer_capacity };
        Buffer number_buffer{ number_buffer_data, 22 };

        return format_raw(&destination_buffer, &number_buffer, a_p_format, args, sizeof...(a_params));
    }

    cstring()               = delete;
    cstring(cstring&&)      = delete;
    cstring(const cstring&) = delete;
    ~cstring()              = delete;

    cstring& operator = (cstring&&)      = delete;
    cstring& operator = (const cstring&) = delete;

private:

    struct Buffer
    {
        char* p_data          = nullptr;
        const uint32 capacity = 0;
    };

    class Argument
    {
    public:

        Argument()  = default;
        ~Argument() = default;

        Argument& operator = (Argument&&)      = delete;
        Argument& operator = (const Argument&) = delete;

        Argument(Argument&& a_other)
            : type(a_other.type)
        {
            memory::copy(this->data, a_other.data, sizeof(a_other.data));
        }

        Argument(const Argument& a_other)
            : type(a_other.type)
        {
            memory::copy(this->data, a_other.data, sizeof(a_other.data));
        }

        explicit Argument(unsigned long int a_value)
            : data{ static_cast<uint8>((a_value >> 24u) & 0xFF),
                    static_cast<uint8>((a_value >> 16u) & 0xFF),
                    static_cast<uint8>((a_value >> 8u) & 0xFF),
                    static_cast<uint8>((a_value >> 0u) & 0xFF) }
            , type(Type::unsigned_int)
        {
            static_assert(sizeof(a_value)    == 4);
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(long int a_value)
            : data{ static_cast<uint8>((a_value >> 24u) & 0xFF),
                    static_cast<uint8>((a_value >> 16u) & 0xFF),
                    static_cast<uint8>((a_value >> 8u) & 0xFF),
                    static_cast<uint8>((a_value >> 0u) & 0xFF) }
            , type(Type::signed_int)
        {
            static_assert(sizeof(a_value)    == 4);
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(uint32 a_value)
            : data { static_cast<uint8>((a_value >> 24u) & 0xFF),
                     static_cast<uint8>((a_value >> 16u) & 0xFF),
                     static_cast<uint8>((a_value >> 8u)  & 0xFF),
                     static_cast<uint8>((a_value >> 0u)  & 0xFF) }
            , type(Type::unsigned_int)
        {
            static_assert(sizeof(a_value)    == 4);
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(int32 a_value)
            : data{ static_cast<uint8>((a_value >> 24u) & 0xFF),
                    static_cast<uint8>((a_value >> 16u) & 0xFF),
                    static_cast<uint8>((a_value >> 8u)  & 0xFF),
                    static_cast<uint8>((a_value >> 0u)  & 0xFF) }
            , type(Type::signed_int)
        {
            static_assert(sizeof(a_value)    == 4);
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(uint16 a_value)
            : data{ 0u,
                    0u,
                    static_cast<uint8>((a_value >> 8u) & 0xFF),
                    static_cast<uint8>((a_value >> 0u) & 0xFF) }
            , type(Type::unsigned_int)
        {
            static_assert(sizeof(a_value)    == 2);
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(int16 a_value)
            : data{ 0u,
                    0u,
                    static_cast<uint8>((a_value >> 8u) & 0xFF),
                    static_cast<uint8>((a_value >> 0u) & 0xFF) }
            , type(Type::signed_int)
        {
            static_assert(sizeof(a_value)    == 2);
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(uint8 a_value)
            : data{ static_cast<uint8>(a_value), 0u, 0u, 0u }
            , type(Argument::Type::unsigned_int)
        {
            static_assert(sizeof(a_value)    == 1);
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(int8 a_value)
            : data{ static_cast<uint8>(a_value), 0u, 0u, 0u }
            , type(Argument::Type::character)
        {
            static_assert(sizeof(a_value)    == 1);
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(const char* a_p_value)
            : data{ static_cast<uint8>((reinterpret_cast<uint32>(a_p_value) >> 24u) & 0xFF),
                    static_cast<uint8>((reinterpret_cast<uint32>(a_p_value) >> 16u) & 0xFF),
                    static_cast<uint8>((reinterpret_cast<uint32>(a_p_value) >> 8u)  & 0xFF),
                    static_cast<uint8>((reinterpret_cast<uint32>(a_p_value) >> 0u)  & 0xFF) }
            , type(Type::cstring)
        {
            static_assert(sizeof(this->data) == 4);
        }

        uint32 get_uint32() const
        {
            assert(this->type == Type::unsigned_int);
            return (this->data[0] << 24u) | (this->data[1] << 16u) | (this->data[2] << 8u) | (this->data[3] << 0u);
        }

        int32 get_int32() const
        {
            assert(this->type == Type::signed_int);
            return (this->data[0] << 24u) | (this->data[1] << 16u) | (this->data[2] << 8u) | (this->data[3] << 0u);
        }

        char get_char() const
        {
            assert(this->type == Type::character);
            return static_cast<char>(this->data[0]);
        }

        const char* get_cstring() const
        {
            assert(this->type == Type::cstring);
            return reinterpret_cast<const char*>((this->data[0] << 24u) |
                                                 (this->data[1] << 16u) |
                                                 (this->data[2] << 8u)  |
                                                 (this->data[3] << 0));
        }

    private:

        enum class Type
        {
            unsigned_int,
            signed_int,
            character,
            cstring,
            unknown,
        };

    private:

        byte data[sizeof(uint32)] = { 0 };
        Type type = Type::unknown;
    };

private:

    static uint32 format_raw(Buffer* a_p_destinaition_buffer,
                             Buffer* a_p_number_buffer,
                             const char* a_p_format,
                             const Argument* a_p_argv,
                             uint32 a_argc);
};

} // namespace common
} // namespace cml