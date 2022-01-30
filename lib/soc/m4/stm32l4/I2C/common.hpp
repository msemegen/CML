#pragma once

/*
 *   Name: common.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// std
#include <cstdint>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
inline bool is_error(I2C_TypeDef* a_p_registers)
{
    return cml::bit::is_any(a_p_registers->ISR,
                            I2C_ISR_TIMEOUT | I2C_ISR_PECERR | I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR |
                                I2C_ISR_NACKF);
}

template<typename Bus_flag_t> Bus_flag_t get_Bus_status_flag(I2C_TypeDef* p_registers)
{
    Bus_flag_t ret          = Bus_flag_t::ok;
    const std::uint32_t isr = p_registers->ISR;

    if (true == cml::bit_flag::is(isr, I2C_ISR_OVR))
    {
        ret |= Bus_flag_t::buffer_error;
    }

    if (true == cml::bit_flag::is(isr, I2C_ISR_ARLO))
    {
        ret |= Bus_flag_t::arbitration_lost;
    }

    if (true == cml::bit_flag::is(isr, I2C_ISR_BERR))
    {
        ret |= Bus_flag_t::misplaced;
    }

    if (true == cml::bit_flag::is(isr, I2C_ISR_NACKF))
    {
        ret |= Bus_flag_t::nack;
    }

    return ret;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc