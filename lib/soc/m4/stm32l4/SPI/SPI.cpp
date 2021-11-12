/*
 *   Name: SPI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/SPI/SPI.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;
using namespace cml::utils;

void SPI_master::enable(const Enable_config& a_config, const Frame_format& a_frame_format)
{
    cml_assert(various::get_enum_incorrect_value<Enable_config::Wiring>() != a_config.wiring);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Crc>() != a_config.crc);
    cml_assert(various::get_enum_incorrect_value<Enable_config::NSS_management>() != a_config.nss_management);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Bit_significance>() != a_frame_format.bit_significance);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Phase>() != a_frame_format.phase);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Polarity>() != a_frame_format.polarity);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Word_length>() != a_frame_format.word_length);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Clock_prescaler>() != a_config.clock_prescaler);

    // not implemented Config::Wiring::half_duplex and Config::Wiring::Simplex
    cml_assert(Enable_config::Wiring::full_duplex == a_config.wiring);

    this->p_registers->CR2 = static_cast<std::uint32_t>(a_frame_format.word_length) | SPI_CR2_SSOE |
                             (a_frame_format.word_length <= Frame_format::Word_length::_8 ? SPI_CR2_FRXTH : 0x0u) |
                             (Enable_config::NSS_management::hardware == a_config.nss_management ? SPI_CR2_SSOE : 0x0u);

    this->p_registers->CR1 =
        SPI_CR1_MSTR | static_cast<std::uint32_t>(a_config.wiring) | static_cast<std::uint32_t>(a_frame_format.phase) |
        static_cast<std::uint32_t>(a_frame_format.polarity) |
        static_cast<std::uint32_t>(a_frame_format.bit_significance) |
        static_cast<std::uint32_t>(a_config.clock_prescaler) |
        (Enable_config::Crc::enable == a_config.crc ? SPI_CR1_CRCEN : 0x0u) |
        (Enable_config::NSS_management::software == a_config.nss_management ? SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE :
                                                                              0x0u);
}

void SPI_master::disable()
{
    if (false == bit_flag::is(p_registers->CR1, SPI_CR1_BIDIMODE) &&
        true == bit_flag::is(p_registers->CR1, SPI_CR1_RXONLY))
    {
        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);
        wait_until::all_bits(&(p_registers->SR), SPI_SR_BSY, true);

        while (true == bit::is_any(p_registers->SR, SPI_SR_FRLVL))
        {
            volatile std::uint32_t dr = p_registers->DR;
            unused(dr);
        }
    }
    else
    {
        wait_until::any_bit(&(p_registers->SR), SPI_SR_FRLVL, true);
        wait_until::all_bits(&(p_registers->SR), SPI_SR_BSY, true);

        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);

        while (true == bit::is_any(p_registers->SR, SPI_SR_FRLVL))
        {
            volatile std::uint32_t dr = p_registers->DR;
            unused(dr);
        }
    }

    p_registers->CR2 = 0;
    p_registers->CR1 = 0;
}

void SPI_slave::enable(const Enable_config& a_config, const Frame_format& a_frame_format)
{
    cml_assert(various::get_enum_incorrect_value<Enable_config::Wiring>() != a_config.wiring);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Crc>() != a_config.crc);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Bit_significance>() != a_frame_format.bit_significance);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Phase>() != a_frame_format.phase);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Polarity>() != a_frame_format.polarity);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Word_length>() != a_frame_format.word_length);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Clock_prescaler>() != a_config.clock_prescaler);

    // not implemented Config::Wiring::half_duplex and Config::Wiring::Simplex
    cml_assert(Enable_config::Wiring::full_duplex == a_config.wiring);

    p_registers->CR2 = static_cast<std::uint32_t>(a_frame_format.word_length) | SPI_CR2_SSOE |
                       (a_frame_format.word_length <= Frame_format::Word_length::_8 ? SPI_CR2_FRXTH : 0x0u);

    p_registers->CR1 = static_cast<std::uint32_t>(a_config.wiring) | static_cast<std::uint32_t>(a_frame_format.phase) |
                       static_cast<std::uint32_t>(a_frame_format.polarity) |
                       static_cast<std::uint32_t>(a_frame_format.bit_significance) |
                       static_cast<std::uint32_t>(a_config.clock_prescaler) |
                       (Enable_config::Crc::enable == a_config.crc ? SPI_CR1_CRCEN : 0x0u) | SPI_CR1_SPE;
}

void SPI_slave::disable()
{
    if (false == bit_flag::is(p_registers->CR1, SPI_CR1_BIDIMODE) &&
        true == bit_flag::is(p_registers->CR1, SPI_CR1_RXONLY))
    {
        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);
        wait_until::all_bits(&(p_registers->SR), SPI_SR_BSY, true);

        while (true == bit::is_any(p_registers->SR, SPI_SR_FRLVL))
        {
            volatile std::uint32_t dr = p_registers->DR;
            unused(dr);
        }
    }
    else
    {
        wait_until::any_bit(&(p_registers->SR), SPI_SR_FRLVL, true);
        wait_until::all_bits(&(p_registers->SR), SPI_SR_BSY, true);

        bit_flag::clear(&(p_registers->CR1), SPI_CR1_SPE);

        while (true == bit::is_any(p_registers->SR, SPI_SR_FRLVL))
        {
            volatile std::uint32_t dr = p_registers->DR;
            unused(dr);
        }
    }

    p_registers->CR2 = 0;
    p_registers->CR1 = 0;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif