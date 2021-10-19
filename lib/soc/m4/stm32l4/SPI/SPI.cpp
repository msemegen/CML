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

template<> void rcc<SPI_base>::enable<SPI1_BASE>(Handle<SPI1_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN);
    }
}

template<> void rcc<SPI_base>::disable<SPI1_BASE>(Handle<SPI1_BASE>)
{
    bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);
    bit_flag::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN);
}

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<SPI_base>::enable<SPI2_BASE>(Handle<SPI2_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI2SMEN);
    }
}

template<> void rcc<SPI_base>::disable<SPI2_BASE>(Handle<SPI2_BASE>)
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN);
    bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI2SMEN);
}
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<SPI_base>::enable<SPI3_BASE>(Handle<SPI3_BASE>, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI3EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI3SMEN);
    }
}

template<> void rcc<SPI_base>::disable<SPI3_BASE>(Handle<SPI3_BASE>)
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI3EN);
    bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI3SMEN);
}
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif