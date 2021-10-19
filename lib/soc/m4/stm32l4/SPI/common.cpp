/*
 *   Name: common.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/SPI/common.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void clear_errors(SPI_TypeDef* p_registers)
{
    const std::uint32_t sr = p_registers->SR;

    if (true == cml::bit_flag::is(sr, SPI_SR_OVR))
    {
        volatile std::uint32_t t1 = p_registers->DR;
        volatile std::uint32_t t2 = p_registers->SR;

        unused(t1);
        unused(t2);
    }

    if (true == bit_flag::is(sr, SPI_SR_MODF))
    {
        volatile std::uint32_t t1 = p_registers->SR;
        unused(t1);

        bit_flag::set(&(p_registers->CR1), SPI_CR1_SPE);
    }

    if (true == bit_flag::is(sr, SPI_SR_FRE))
    {
        volatile std::uint32_t t1 = p_registers->SR;
        unused(t1);
    }

    if (true == bit_flag::is(sr, SPI_SR_CRCERR))
    {
        bit::clear(&(p_registers->SR), SPI_SR_CRCERR_Pos);
    }
}

} // namespace stm32l4
} // namespace m4
} // namespace soc