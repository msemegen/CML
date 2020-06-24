#pragma once

/*
    Name: crc32.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l452xx.h>

//cml
#include <cml/integer.hpp>
#include <cml/bit.hpp>

namespace soc {
namespace stm32l452xx {
namespace system {

class crc32
{
public:

    enum class In_data_reverse : cml::uint32
    {
        none      = 0,
        byte      = CRC_CR_REV_IN_0,
        half_word = CRC_CR_REV_IN_1,
        word      = CRC_CR_REV_IN_0 | CRC_CR_REV_IN_1
    };

    enum class Out_data_reverse : cml::uint32
    {
        none    = 0,
        enabled = CRC_CR_REV_OUT
    };

    static void enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse);
    static void disable();

    static void update_uint8(cml::uint8 a_value)
    {
        CRC->DR = a_value;
    }

    static void update_uint16(cml::uint16 a_value)
    {
        CRC->DR = a_value;
    }

    static void update_uint32(cml::uint32 a_value)
    {
        CRC->DR = a_value;
    }

    static cml::uint32 get_value()
    {
        return CRC->DR;
    }

    static void reset()
    {
        cml::set_flag(&(CRC->CR), CRC_CR_RESET);
    }

private:

    crc32()             = delete;
    crc32(crc32&&)      = delete;
    crc32(const crc32&) = delete;
    ~crc32()            = default;

    crc32& operator = (crc32&&)      = delete;
    crc32& operator = (const crc32&) = delete;
};

} // namespace system
} // namespace stm32l452xx
} // namespace soc