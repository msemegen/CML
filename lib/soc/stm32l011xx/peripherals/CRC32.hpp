#pragma once

/*
 *   Name: CRC32.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l0xx.h>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

class CRC32 : private cml::Non_copyable
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

public:
    CRC32();
    ~CRC32();

    void enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse);
    void disable();

    uint32_t calculate(const uint8_t* a_p_data, uint32_t a_data_size);
};

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc