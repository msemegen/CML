#pragma once

/*
    Name: format.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>
#include <common/memory.hpp>
#include <collection/string.hpp>

namespace cml {
namespace common {

class c_format
{
public:

    c_format()                = delete;
    c_format(c_format&&)      = delete;
    c_format(const c_format&) = delete;

    c_format& operator = (c_format&&)      = delete;
    c_format& operator = (const c_format&) = delete;

    template<typename ... types>
    explicit c_format(collection::c_string* a_p_buffer, const char* a_p_format, types ... a_params)
    {
        const c_argument args[] = { c_argument{a_params}... };
        this->raw(a_p_buffer, a_p_format, args, sizeof...(a_params));
    }

private:

    class c_argument
    {
    public:

        c_argument()  = default;
        ~c_argument() = default;

        c_argument& operator = (c_argument&&)      = delete;
        c_argument& operator = (const c_argument&) = delete;

        c_argument(c_argument&& a_other)
            : type(a_other.type)
        {
            memory_copy(this->data, a_other.data, sizeof(a_other.data));
        }

        c_argument(const c_argument& a_other)
            : type(a_other.type)
        {
            memory_copy(this->data, a_other.data, sizeof(a_other.data));
        }

        explicit c_argument(uint32 a_value)
            : data { static_cast<uint8>((a_value >> 24) & 0xFF),
                     static_cast<uint8>((a_value >> 16) & 0xFF),
                     static_cast<uint8>((a_value >> 8 ) & 0xFF),
                     static_cast<uint8>(a_value & 0xFF) }
            , type(c_argument::e_type::unsigned_int)
        {
            static_assert(sizeof(this->data) == 4);
        }

        explicit c_argument(int32 a_value)
            : data{ static_cast<uint8>((a_value >> 24) & 0xFF),
                    static_cast<uint8>((a_value >> 16) & 0xFF),
                    static_cast<uint8>((a_value >> 8) & 0xFF),
                    static_cast<uint8>(a_value & 0xFF) }
            , type(c_argument::e_type::signed_int)
        {
            static_assert(sizeof(this->data) == 4);
        }

        explicit c_argument(int8 a_value)
            : data{ static_cast<uint8>(a_value), 0u, 0u, 0u }
            , type(c_argument::e_type::character)
        {
            static_assert(sizeof(this->data) == 4);
        }

        explicit c_argument(const char* a_p_value)
        {

        }

        uint32 get_uint32() const
        {
            _assert(this->type == c_argument::e_type::unsigned_int);
            return this->data[0] | this->data[1] << 8u | this->data[2] << 16u << this->data[3] << 24;
        }

        int32 get_int32() const
        {
            _assert(this->type == c_argument::e_type::signed_int);
            return this->data[0] | this->data[1] << 8u | this->data[2] << 16u << this->data[3] << 24;
        }

        char get_char() const
        {
            _assert(this->type == c_argument::e_type::character);
            return static_cast<char>(this->data[0]);
        }

    private:

        enum class e_type
        {
            unknown,
            unsigned_int,
            signed_int,
            character,
            cstring
        };

    private:

        byte data[sizeof(uint32)] = { 0 };
        c_argument::e_type type = c_argument::e_type::unknown;
    };

    void raw(collection::c_string* a_p_out_string,
             const char* a_p_format,
             const c_argument* a_p_argv,
             uint32 a_argc);
};

} // namespace common
} // namespace cml