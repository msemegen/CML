#pragma once

/*
    Name: SPI.hpp

    Copyright(c) 2021 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>
#include <type_traits>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/time.hpp>

// soc
#include <soc/stm32l452xx/peripherals/GPIO.hpp>

namespace soc {
namespace stm32l452xx {
namespace peripherals {

class SPI_base : private cml::Non_copyable
{
public:
    enum class Id : uint32_t
    {
        _1,
        _2,
        _3
    };

    struct Frame_format
    {
        enum class Word_length : uint32_t
        {
            _4  = 0x300u,
            _5  = 0x400u,
            _6  = 0x500u,
            _7  = 0x600u,
            _8  = 0x700u,
            _9  = 0x800u,
            _10 = 0x900u,
            _11 = 0xA00u,
            _12 = 0xB00u,
            _13 = 0xC00u,
            _14 = 0xD00u,
            _15 = 0xE00u,
            _16 = 0xF00u,
            unknown
        };

        enum class Bit_significance : uint32_t
        {
            most  = 0x0u,
            least = SPI_CR1_LSBFIRST,
            unknown
        };

        Word_length word_length           = Word_length::unknown;
        Bit_significance bit_significance = Bit_significance::unknown;
    };

    struct Clock_source
    {
        enum class Type
        {
            pclkx,
            unknown
        };

        enum class Prescaler : uint32_t
        {
            _2   = 0x00u,
            _4   = 0x08u,
            _8   = 0x10u,
            _16  = 0x18u,
            _32  = 0x20u,
            _64  = 0x28u,
            _128 = 0x30u,
            _256 = 0x38u,
            unknown
        };

        Type type           = Type::unknown;
        Prescaler prescaler = Prescaler::unknown;
    };

    struct Config
    {
        enum class Wiring : uint32_t
        {
            full_duplex = 0x0000u,
            half_duplex = SPI_CR1_BIDIMODE,
            simplex     = SPI_CR1_RXONLY,
            unknown
        };

        enum class Mode : uint32_t
        {
            _0 = 0x0u,
            _1 = SPI_CR1_CPHA,
            _2 = SPI_CR1_CPOL,
            _3 = SPI_CR1_CPOL | SPI_CR1_CPHA,
            unknown
        };

        Mode mode       = Mode::unknown;
        Wiring wiring   = Wiring::unknown;
        bool crc_enable = false;
    };

    struct Result
    {
        enum class Bus_flag : uint32_t
        {
            ok          = 0x0u,
            overrun     = 0x1u,
            crc_error   = 0x2u,
            frame_error = 0x4u,
            mode_fault  = 0x8u,
            unknown     = 0x10u
        };

        Bus_flag bus_flag             = Bus_flag::unknown;
        uint32_t data_length_in_words = 0;
    };

public:
    Id get_id() const
    {
        return this->id;
    }

    Config get_config() const
    {
        return this->confg;
    }

    Frame_format get_frame_format() const
    {
        return this->frame_format;
    }

    Clock_source get_clock_source() const
    {
        return this->clock_source;
    }

protected:
    SPI_base(Id a_id)
        : id(a_id)
    {
    }

protected:
    Id id;

    Config confg;
    Frame_format frame_format;
    Clock_source clock_source;
};

constexpr SPI_base::Result::Bus_flag operator|(SPI_base::Result::Bus_flag a_f1, SPI_base::Result::Bus_flag a_f2)
{
    return static_cast<SPI_base::Result::Bus_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr SPI_base::Result::Bus_flag operator&(SPI_base::Result::Bus_flag a_f1, SPI_base::Result::Bus_flag a_f2)
{
    return static_cast<SPI_base::Result::Bus_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr SPI_base::Result::Bus_flag operator|=(SPI_base::Result::Bus_flag& a_f1, SPI_base::Result::Bus_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

class SPI_master : public SPI_base
{
public:
    using Id           = SPI_base::Id;
    using Frame_format = SPI_base::Frame_format;
    using Clock_source = SPI_base::Clock_source;
    using Config       = SPI_base::Config;
    using Result       = SPI_base::Result;

public:
    SPI_master(Id a_id)
        : SPI_base(a_id)
    {
    }

    void enable(const Config& a_config,
                const Frame_format& a_frame_format,
                const Clock_source& a_clock_source,
                uint32_t a_irq_priority);
    void disable();

    template<typename Data_t> Result transmit_polling(const Data_t& a_data, GPIO::Out::Pin* a_p_nss = nullptr)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data), a_p_nss);
    }

    template<typename Data_t>
    Result transmit_polling(const Data_t& a_data, cml::time::tick a_timeout, GPIO::Out::Pin* a_p_nss = nullptr)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data), a_timeout, a_p_nss);
    }

    Result
    transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss = nullptr);
    Result transmit_bytes_polling(const void* a_p_data,
                                  uint32_t a_data_size_in_words,
                                  cml::time::tick a_timeout,
                                  GPIO::Out::Pin* a_p_nss = nullptr);

    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss = nullptr);
    Result receive_bytes_polling(void* a_p_data,
                                 uint32_t a_data_size_in_words,
                                 cml::time::tick a_timeout,
                                 GPIO::Out::Pin* a_p_nss = nullptr);
};

class SPI_slave : public SPI_base
{
public:
    using Id           = SPI_base::Id;
    using Frame_format = SPI_base::Frame_format;
    using Clock_source = SPI_base::Clock_source;
    using Config       = SPI_base::Config;

public:
    SPI_slave(Id a_id)
        : SPI_base(a_id)
    {
    }

    void enable(const Config& a_config,
                const Frame_format& a_frame_format,
                const Clock_source& a_clock_source,
                uint32_t a_irq_priority);
    void disable();
};

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc