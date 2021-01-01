#pragma once

/*
    Name: crc32.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// externals
#include <stm32l0xx.h>

// cml
#include <cml/bit_flag.hpp>

namespace soc {
namespace stm32l011xx {
namespace system {

class crc32
{
public:
    enum class In_data_reverse : uint32_t
    {
        none      = 0,
        byte      = CRC_CR_REV_IN_0,
        half_word = CRC_CR_REV_IN_1,
        word      = CRC_CR_REV_IN_0 | CRC_CR_REV_IN_1
    };

    enum class Out_data_reverse : uint32_t
    {
        none    = 0,
        enabled = CRC_CR_REV_OUT
    };

    static void enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse);
    static void disable();

    void update_uint8(uint8_t a_value)
    {
        CRC->DR = a_value;
    }

    void update_uint16(uint16_t a_value)
    {
        CRC->DR = a_value;
    }

    void update_uint32(uint32_t a_value)
    {
        CRC->DR = a_value;
    }

    uint32_t get_value() const
    {
        return CRC->DR;
    }

    void reset()
    {
        cml::bit_flag::set(&(CRC->CR), CRC_CR_RESET);
    }

private:
    crc32()             = delete;
    crc32(crc32&&)      = delete;
    crc32(const crc32&) = delete;
    ~crc32()            = default;

    crc32& operator=(crc32&&) = delete;
    crc32& operator=(const crc32&) = delete;
};

} // namespace system
} // namespace stm32l011xx
} // namespace soc