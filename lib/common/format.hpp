#pragma once

/*
    Name: format.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <collection/string.hpp>
#include <common/assert.hpp>
#include <common/integer.hpp>
#include <common/memory.hpp>

namespace cml {
namespace common {

class format
{
public:

    format()               = delete;
    format(format&&)      = delete;
    format(const format&) = delete;

    format& operator = (format&&)      = delete;
    format& operator = (const format&) = delete;

    template<typename ... types>
    explicit format(collection::String* a_p_buffer, const char* a_p_format, types ... a_params)
    {
        const Argument args[] = { Argument{a_params}... };
        this->raw(a_p_buffer, a_p_format, args, sizeof...(a_params));
    }

private:

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
            memory_copy(this->data, a_other.data, sizeof(a_other.data));
        }

        Argument(const Argument& a_other)
            : type(a_other.type)
        {
            memory_copy(this->data, a_other.data, sizeof(a_other.data));
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
        {

        }

        uint32 get_uint32() const
        {
            assert(this->type == Type::unsigned_int);
            return this->data[0] | this->data[1] << 8u | this->data[2] << 16u << this->data[3] << 24;
        }

        int32 get_int32() const
        {
            assert(this->type == Type::signed_int);
            return this->data[0] | this->data[1] << 8u | this->data[2] << 16u << this->data[3] << 24;
        }

        char get_char() const
        {
            assert(this->type == Type::character);
            return static_cast<char>(this->data[0]);
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

    void raw(collection::String* a_p_out_string,
             const char* a_p_format,
             const Argument* a_p_argv,
             uint32 a_argc);
};

} // namespace common
} // namespace cml