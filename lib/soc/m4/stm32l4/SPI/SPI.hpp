#pragma once

/*
 *   Name: SPI.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// external
#include <stm32l4xx.h>

// soc
#include <soc/Handle.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class SPI_base : private cml::Non_copyable
{
public:
    struct id : cml::Non_constructible
    {
        constexpr static auto _1 = Handle<SPI1_BASE> {};
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        constexpr static auto _2 = Handle<SPI2_BASE> {};
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        constexpr static auto _3 = Handle<SPI3_BASE> {};
#endif
    };

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

    struct Transmit_callback
    {
        using Function = void (*)(volatile std::uint16_t* a_p_data,
                                  bool a_stop,
                                  SPI_base* a_p_this,
                                  void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Receive_callback
    {
        using Function = void (*)(std::uint16_t a_data, SPI_base* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Transmit_receive_callback
    {
        using Transmit_function = void (*)(volatile std::uint16_t* a_p_data,
                                           bool a_stop,
                                           SPI_base* a_p_this,
                                           void* a_p_user_data);
        using Receive_function  = void (*)(std::uint16_t a_data, SPI_base* a_p_this, void* a_p_user_data);

        Transmit_function transmit = nullptr;
        Receive_function receive   = nullptr;
        void* p_user_data          = nullptr;
    };

public:
    SPI_base(Handle<SPI1_BASE>)
        : idx(0u)
        , p_registers(SPI1)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    SPI_base(Handle<SPI2_BASE>)
        : idx(1u)
        , p_registers(SPI2)
    {
    }
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    SPI_base(Handle<SPI3_BASE>)
        : idx(2u)
        , p_registers(SPI3)
    {
    }
#endif

    std::uint32_t get_id() const
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
    std::uint32_t idx;
    SPI_TypeDef* p_registers;
};

class SPI_master : public SPI_base
{
public:
    using id = SPI_base::id;

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
    SPI_master(Handle<SPI1_BASE>)
        : SPI_base(id::_1)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    SPI_master(Handle<SPI2_BASE>)
        : SPI_base(id::_2)
    {
    }
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    SPI_master(Handle<SPI3_BASE>)
        : SPI_base(id::_3)
    {
    }
#endif

    ~SPI_master()
    {
        this->disable();
    }

    void enable(const Enable_config& a_config, const Frame_format& a_frame_format);
    void disable();

    Enable_config get_Enable_config() const;
    Frame_format get_Frame_format() const;
};

class SPI_slave : public SPI_base
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
    SPI_slave(Handle<SPI1_BASE>)
        : SPI_base(id::_1)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    SPI_slave(Handle<SPI2_BASE>)
        : SPI_base(id::_2)
    {
    }
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    SPI_slave(Handle<SPI3_BASE>)
        : SPI_base(id::_3)
    {
    }
#endif

    ~SPI_slave()
    {
        this->disable();
    }

    void enable(const Enable_config& a_config, const Frame_format& a_frame_format);
    void disable();

    Enable_config get_Enable_config() const;
    Frame_format get_Frame_format() const;
};

template<> class rcc<SPI_base> : private cml::Non_constructible
{
public:
    template<std::uint32_t peripheral_base_address>
    static void enable(Handle<peripheral_base_address>, bool a_enable_in_lp);
    template<std::uint32_t peripheral_base_address> static void disable(Handle<peripheral_base_address>);
};

template<> void rcc<SPI_base>::enable<SPI1_BASE>(Handle<SPI1_BASE>, bool a_enable_in_lp);
template<> void rcc<SPI_base>::disable<SPI1_BASE>(Handle<SPI1_BASE>);

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<SPI_base>::enable<SPI2_BASE>(Handle<SPI2_BASE>, bool a_enable_in_lp);
template<> void rcc<SPI_base>::disable<SPI2_BASE>(Handle<SPI2_BASE>);
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<SPI_base>::enable<SPI3_BASE>(Handle<SPI3_BASE>, bool a_enable_in_lp);
template<> void rcc<SPI_base>::disable<SPI3_BASE>(Handle<SPI3_BASE>);
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc