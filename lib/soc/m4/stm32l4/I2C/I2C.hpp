#pragma once

/*
 *   Name: I2C.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// std
#include <cstdint>

// soc
#include <soc/Handle.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class I2C_base : private cml::Non_copyable
{
public:
    struct id : private cml::Non_constructible
    {
        constexpr static auto _1 = Handle<I2C1_BASE> {};
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        constexpr static auto _2 = Handle<I2C2_BASE> {};
#endif
        constexpr static auto _3 = Handle<I2C3_BASE> {};
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        constexpr static auto _4 = Handle<I2C4_BASE> {};
#endif
    };

public:
    I2C_base(Handle<I2C1_BASE>)
        : idx(0u)
        , p_registers(I2C1)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    I2C_base(Handle<I2C2_BASE>)
        : idx(1u)
        , p_registers(I2C2)
    {
    }
#endif
    I2C_base(Handle<I2C3_BASE>)
        : idx(2u)
        , p_registers(I2C3)
    {
    }
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    I2C_base(Handle<I2C4_BASE>)
        : idx(3)
        , p_registers(I2C4)
    {
    }
#endif

    std::uint32_t get_id() const
    {
        return this->idx;
    }

    operator I2C_TypeDef*()
    {
        return this->p_registers;
    }

    operator const I2C_TypeDef*() const
    {
        return this->p_registers;
    }

protected:
    std::uint32_t idx;
    I2C_TypeDef* p_registers;
};

class I2C_master : public I2C_base
{
public:
    using id = I2C_base::id;

    struct Enable_config
    {
        enum class Analog_filter : std::uint32_t
        {
            disabled,
            enabled
        };

        enum class Fast_plus : std::uint32_t
        {
            disabled,
            enabled
        };

        enum class Crc : std::uint32_t
        {
            disabled,
            enabled
        };

        Analog_filter analog_filter = cml::various::get_enum_incorrect_value<Analog_filter>();
        Fast_plus fast_plus         = cml::various::get_enum_incorrect_value<Fast_plus>();
        Crc crc                     = cml::various::get_enum_incorrect_value<Crc>();
        std::uint32_t timings       = 0;
    };

public:
    I2C_master(Handle<I2C1_BASE>)
        : I2C_base(id::_1)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    I2C_master(Handle<I2C2_BASE>)
        : I2C_base(id::_2)
    {
    }
#endif
    I2C_master(Handle<I2C3_BASE>)
        : I2C_base(id::_3)
    {
    }
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    I2C_master(Handle<I2C4_BASE>)
        : I2C_base(id::_4)
    {
    }
#endif

    ~I2C_master();

    void enable(const Enable_config& a_config);
    void disable();

    Enable_config get_Enable_config() const;
};

class I2C_slave : public I2C_base
{
public:
    using id = I2C_base::id;

    struct Enable_config
    {
        enum class Analog_filter : std::uint32_t
        {
            disabled,
            enabled,
        };

        enum class Fast_plus : std::uint32_t
        {
            disabled,
            enabled,
        };

        enum class Crc : std::uint32_t
        {
            disabled,
            enabled,
        };

        Analog_filter analog_filter = cml::various::get_enum_incorrect_value<Analog_filter>();
        Fast_plus fast_plus         = cml::various::get_enum_incorrect_value<Fast_plus>();
        Crc crc                     = cml::various::get_enum_incorrect_value<Crc>();
        std::uint32_t timings       = 0;
        std::uint16_t address       = 0;
    };

public:
    I2C_slave(Handle<I2C1_BASE>)
        : I2C_base(id::_1)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    I2C_slave(Handle<I2C2_BASE>)
        : I2C_base(id::_2)
    {
    }
#endif
    I2C_slave(Handle<I2C3_BASE>)
        : I2C_base(id::_3)
    {
    }
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    I2C_slave(Handle<I2C4_BASE>)
        : I2C_base(id::_4)
    {
    }
#endif

    ~I2C_slave();

    void enable(const Enable_config& a_config);
    void disable();

    Enable_config get_Enable_config() const;
};

template<> class rcc<I2C_base> : private cml::Non_constructible
{
public:
    enum class Clock_source : uint32_t
    {
        PCLK1  = 0,
        SYSCLK = 1,
        HSI    = 2
    };

    template<Clock_source, std::uint32_t peripheral_base_address>
    static void enable(Handle<peripheral_base_address>, bool a_enable_in_lp);
    template<std::uint32_t peripheral_base_address> static void disable(Handle<peripheral_base_address>);
};

template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::HSI, I2C1_BASE>(Handle<I2C1_BASE>, bool a_enable_in_lp);
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::PCLK1, I2C1_BASE>(Handle<I2C1_BASE>, bool a_enable_in_lp);
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::SYSCLK, I2C1_BASE>(Handle<I2C1_BASE>, bool a_enable_in_lp);
template<> void rcc<I2C_base>::disable<I2C1_BASE>(Handle<I2C1_BASE>);
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::HSI, I2C2_BASE>(Handle<I2C2_BASE>, bool a_enable_in_lp);
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::PCLK1, I2C2_BASE>(Handle<I2C2_BASE>, bool a_enable_in_lp);
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::SYSCLK, I2C2_BASE>(Handle<I2C2_BASE>, bool a_enable_in_lp);
template<> void rcc<I2C_base>::disable<I2C2_BASE>(Handle<I2C2_BASE>);
#endif
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::HSI, I2C3_BASE>(Handle<I2C3_BASE>, bool a_enable_in_lp);
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::PCLK1, I2C3_BASE>(Handle<I2C3_BASE>, bool a_enable_in_lp);
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::SYSCLK, I2C3_BASE>(Handle<I2C3_BASE>, bool a_enable_in_lp);
template<> void rcc<I2C_base>::disable<I2C3_BASE>(Handle<I2C3_BASE>);
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::HSI, I2C4_BASE>(Handle<I2C4_BASE>, bool a_enable_in_lp);
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::PCLK1, I2C4_BASE>(Handle<I2C4_BASE>, bool a_enable_in_lp);
template<>
void rcc<I2C_base>::enable<rcc<I2C_base>::Clock_source::SYSCLK, I2C4_BASE>(Handle<I2C4_BASE>, bool a_enable_in_lp);
template<> void rcc<I2C_base>::disable<I2C4_BASE>(Handle<I2C4_BASE>);
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc