#pragma once

/*
    Name: cstring.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>
#include <common/memory.hpp>

namespace cml {
namespace common {

struct cstring
{
    static uint32 length(const char* a_p_string, uint32 a_max_length);
    static bool the_same(const char* a_p_string_1, const char* a_p_string_2, uint32 a_max_length);

    static uint32 join(char* a_p_destination,
                       uint32 a_destination_capacity,
                       const char* a_p_source,
                       uint32 a_source_length);

    template<typename type>
    static type dec_to_integer(const char* a_p_string)
    {
        assert(nullptr != a_p_string && 0 != a_p_string[0]);

        type retval = 0;

        const type min = '-' == a_p_string[0] ? 0 : 1;
        const uint32 length = cstring::length(a_p_string, 22);

        for (uint32 i = length - 1, m = 1; i + length + min != length; i--, m *= 10)
        {
            assert(a_p_string[i] >= '0' && a_p_string[i] <= '9');
            retval += (a_p_string[i] - '0') * m;
        }

        return retval * ('-' == a_p_string[0] ? -1 : 1);
    }

    template<typename type>
    static uint32 from_dec_integer(type a_value, char* a_p_buffer, uint32 a_buffer_capacity)
    {
        uint32 index  = 0;
        uint32 start  = 0;
        uint32 length = 0;

        type temp = a_value;

        if (a_value < 0)
        {
            assert(a_buffer_capacity > 1);

            a_p_buffer[0] = '-';
            a_p_buffer[1] = 0;
            length = 1;
            start  = 1;
        }
        else
        {
            assert(a_buffer_capacity > 0);

            a_p_buffer[index] = 0;
        }

        while (0 != temp)
        {
            char v = static_cast<char>(temp % 10);
            char to_insert = v < 0 ? v * -1 + '0' : v + '0';

            assert(to_insert >= '0' && to_insert <= '9');

            memory::copy(a_p_buffer + 1, a_p_buffer, ++length);
            a_p_buffer[start] = to_insert;

            temp /= 10;
        }

        a_p_buffer[length] = 0;

        return length;
    }

    template<typename ... types>
    static uint32 format(char* a_p_buffer, uint32 a_buffer_capacity, const char* a_p_format, types ... a_params)
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
        char* p_data;
        const uint32 capacity;
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

        explicit Argument(uint32 a_value)
            : data { static_cast<uint8>((a_value >> 24) & 0xFF),
                     static_cast<uint8>((a_value >> 16) & 0xFF),
                     static_cast<uint8>((a_value >> 8 ) & 0xFF),
                     static_cast<uint8>(a_value & 0xFF) }
            , type(Type::unsigned_int)
        {
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(int32 a_value)
            : data{ static_cast<uint8>((a_value >> 24) & 0xFF),
                    static_cast<uint8>((a_value >> 16) & 0xFF),
                    static_cast<uint8>((a_value >> 8) & 0xFF),
                    static_cast<uint8>(a_value & 0xFF) }
            , type(Type::signed_int)
        {
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(int8 a_value)
            : data{ static_cast<uint8>(a_value), 0u, 0u, 0u }
            , type(Argument::Type::character)
        {
            static_assert(sizeof(this->data) == 4);
        }

        explicit Argument(const char* a_p_value)
            : data{ static_cast<uint8>((reinterpret_cast<uint32>(a_p_value) >> 24) & 0xFF),
                    static_cast<uint8>((reinterpret_cast<uint32>(a_p_value) >> 16) & 0xFF),
                    static_cast<uint8>((reinterpret_cast<uint32>(a_p_value) >> 8) & 0xFF),
                    static_cast<uint8>((reinterpret_cast<uint32>(a_p_value) & 0xFF)) }
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