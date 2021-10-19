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
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
void clear_errors(SPI_TypeDef* p_registers);

template<typename Bus_flag_t> Bus_flag_t get_Bus_flag(SPI_TypeDef* p_registers)
{
    Bus_flag_t ret   = Bus_flag_t::ok;
    std::uint32_t sr = p_registers->SR;

    if (true == cml::bit_flag::is(sr, SPI_SR_OVR))
    {
        ret |= Bus_flag_t::overrun;
    }

    if (true == cml::bit_flag::is(sr, SPI_SR_FRE))
    {
        ret |= Bus_flag_t::frame_error;
    }

    if (true == cml::bit_flag::is(sr, SPI_SR_MODF))
    {
        ret |= Bus_flag_t::mode_fault;
    }

    if (true == cml::bit_flag::is(sr, SPI_SR_CRCERR))
    {
        ret |= Bus_flag_t::crc_error;
    }

    return ret;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc
