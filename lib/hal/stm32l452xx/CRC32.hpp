#pragma once

/*
    Name: CRC32.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l452xx.h>

//cml
#include <common/integer.hpp>
#include <common/bit.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class CRC32
{
public:

    enum class In_data_reverse : common::uint32
    {
        none      = 0,
        byte      = CRC_CR_REV_IN_0,
        half_word = CRC_CR_REV_IN_1,
        word      = CRC_CR_REV_IN_0 | CRC_CR_REV_IN_1
    };

    enum class Out_data_reverse : common::uint32
    {
        none    = 0,
        enabled = CRC_CR_REV_OUT
    };

    void enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse);
    void disable();

    void update_uint8(common::uint8 a_value)
    {
        this->p_crc->DR = a_value;
    }

    void update_uint16(common::uint16 a_value)
    {
        this->p_crc->DR = a_value;
    }

    void update_uint32(common::uint32 a_value)
    {
        this->p_crc->DR = a_value;
    }

    common::uint32 get_value() const
    {
        return this->p_crc->DR;
    }

    void reset()
    {
        common::set_flag(&(this->p_crc->CR), CRC_CR_RESET);
    }

    static CRC32& get_instance()
    {
        static CRC32 instance;
        return instance;
    }

private:

    CRC32()
        : p_crc(CRC)
    {}

    CRC32(CRC32&&)      = delete;
    CRC32(const CRC32&) = delete;

    CRC32& operator = (CRC32&&)      = delete;
    CRC32& operator = (const CRC32&) = delete;

private:

    CRC_TypeDef *p_crc;
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml