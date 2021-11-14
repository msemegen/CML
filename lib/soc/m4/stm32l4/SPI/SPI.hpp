#pragma once

/*
 *   Name: SPI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <tuple>

// external
#include <stm32l4xx.h>

// soc
#include <soc/Factory.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class SPI : private cml::Non_copyable
{
public:
    struct Frame_format
    {
        enum class Word_length : std::uint32_t
        {
            _4  = SPI_CR2_DS_0 | SPI_CR2_DS_1,
            _5  = SPI_CR2_DS_2,
            _6  = SPI_CR2_DS_0 | SPI_CR2_DS_2,
            _7  = SPI_CR2_DS_1 | SPI_CR2_DS_2,
            _8  = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2,
            _9  = SPI_CR2_DS_3,
            _10 = SPI_CR2_DS_0 | SPI_CR2_DS_3,
            _11 = SPI_CR2_DS_1 | SPI_CR2_DS_3,
            _12 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_3,
            _13 = SPI_CR2_DS_2 | SPI_CR2_DS_3,
            _14 = SPI_CR2_DS_0 | SPI_CR2_DS_2 | SPI_CR2_DS_3,
            _15 = SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3,
            _16 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3,
        };

        enum class Bit_significance : std::uint32_t
        {
            most  = 0x0u,
            least = SPI_CR1_LSBFIRST,
        };

        enum class Polarity : std::uint32_t
        {
            low  = 0x0u,
            high = SPI_CR1_CPOL
        };

        enum class Phase : std::uint32_t
        {
            first_edge  = 0x0u,
            second_edge = SPI_CR1_CPHA,
        };

        Polarity polarity                 = cml::various::get_enum_incorrect_value<Polarity>();
        Phase phase                       = cml::various::get_enum_incorrect_value<Phase>();
        Word_length word_length           = cml::various::get_enum_incorrect_value<Word_length>();
        Bit_significance bit_significance = cml::various::get_enum_incorrect_value<Bit_significance>();
    };

public:
    std::uint32_t get_idx() const
    {
        return this->idx;
    }

    operator SPI_TypeDef*()
    {
        return this->p_registers;
    }

    operator const SPI_TypeDef*() const
    {
        return this->p_registers;
    }

protected:
    SPI(std::size_t a_idx, SPI_TypeDef* a_p_SPI)
        : idx(a_idx)
        , p_registers(a_p_SPI)
    {
    }

protected:
    const std::uint32_t idx;
    SPI_TypeDef* p_registers;
};

class SPI_master : public SPI
{
public:
    struct Enable_config
    {
        enum class Clock_prescaler : std::uint32_t
        {
            _2   = 0x00u,
            _4   = 0x08u,
            _8   = 0x10u,
            _16  = 0x18u,
            _32  = 0x20u,
            _64  = 0x28u,
            _128 = 0x30u,
            _256 = 0x38u,
        };

        enum class Wiring : std::uint32_t
        {
            full_duplex = 0x0000u,
            simplex     = SPI_CR1_RXONLY,
            half_duplex = SPI_CR1_BIDIMODE
        };

        enum class NSS_management : std::uint32_t
        {
            hardware,
            software = SPI_CR1_SSM | SPI_CR1_SSI
        };

        enum class Crc : std::uint32_t
        {
            disable = 0x0u,
            enable  = SPI_CR1_CRCEN
        };

        Clock_prescaler clock_prescaler = cml::various::get_enum_incorrect_value<Clock_prescaler>();
        Wiring wiring                   = cml::various::get_enum_incorrect_value<Wiring>();
        NSS_management nss_management   = cml::various::get_enum_incorrect_value<NSS_management>();
        Crc crc                         = cml::various::get_enum_incorrect_value<Crc>();
    };

public:
    ~SPI_master()
    {
        this->disable();
    }

    void enable(const Enable_config& a_config, const Frame_format& a_frame_format);
    void disable();

    Enable_config get_Enable_config() const;
    Frame_format get_Frame_format() const;

private:
    SPI_master(std::size_t a_idx, SPI_TypeDef* a_p_SPI)
        : SPI(a_idx, a_p_SPI)
    {
    }

private:
    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

class SPI_slave : public SPI
{
public:
    struct Enable_config
    {
        enum class Clock_prescaler : std::uint32_t
        {
            _2   = 0x00u,
            _4   = 0x08u,
            _8   = 0x10u,
            _16  = 0x18u,
            _32  = 0x20u,
            _64  = 0x28u,
            _128 = 0x30u,
            _256 = 0x38u,
        };

        enum class Wiring : std::uint32_t
        {
            full_duplex = 0x0000u,
            simplex     = SPI_CR1_RXONLY,
            half_duplex = SPI_CR1_BIDIMODE
        };

        enum class Crc : std::uint32_t
        {
            disable = 0x0u,
            enable  = SPI_CR1_CRCEN,
        };

        Clock_prescaler clock_prescaler = cml::various::get_enum_incorrect_value<Clock_prescaler>();
        Wiring wiring                   = cml::various::get_enum_incorrect_value<Wiring>();
        Crc crc                         = cml::various::get_enum_incorrect_value<Crc>();
    };

public:
    ~SPI_slave()
    {
        this->disable();
    }

    void enable(const Enable_config& a_config, const Frame_format& a_frame_format);
    void disable();

    Enable_config get_Enable_config() const;
    Frame_format get_Frame_format() const;

private:
    SPI_slave(std::size_t a_idx, SPI_TypeDef* a_p_SPI)
        : SPI(a_idx, a_p_SPI)
    {
    }

private:
    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

template<std::size_t id> class rcc<SPI, id> : private cml::Non_constructible
{
public:
    static void enable(bool a_enable_in_lp) = delete;
    static void disable()                   = delete;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc