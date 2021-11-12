#pragma once

/*
 *   Name: Basic_timer.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <tuple>

// externals
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
class Basic_timer : private cml::Non_copyable
{
public:
    struct id : private cml::Non_constructible
    {
        static constexpr auto _6 = Handle<TIM6_BASE> {};
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
        static constexpr auto _7 = Handle<TIM7_BASE> {};
#endif
    };

    struct Enable_config
    {
        enum class Mode : std::uint32_t
        {
            disabled,
            enabled
        };

        enum class Autoreload_preload : std::uint32_t
        {
            disabled = 0x0u,
            enabled  = TIM_CR1_ARPE,
        };

        Mode mode                              = cml::various::get_enum_incorrect_value<Mode>();
        Autoreload_preload auto_reload_preload = cml::various::get_enum_incorrect_value<Autoreload_preload>();

        std::uint16_t prescaler   = 0u;
        std::uint16_t auto_reload = 0u;
    };

public:
    Basic_timer(Handle<TIM6_BASE>)
        : idx(0u)
        , irqn { IRQn_Type::TIM6_IRQn }
        , p_registers(TIM6)
    {
    }
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
    Basic_timer(Handle<TIM7_BASE>)
        : idx(1u)
        , irqn { IRQn_Type::TIM7_IRQn }
        , p_registers(TIM7)
    {
    }
#endif
    ~Basic_timer()
    {
        this->stop();
        this->disable();
    }

    void enable(const Enable_config& a_config);
    void disable();

    void start();
    void stop();

    std::uint32_t get_idx() const
    {
        return this->idx;
    }

    operator TIM_TypeDef*()
    {
        return this->p_registers;
    }

    operator const TIM_TypeDef*() const
    {
        return this->p_registers;
    }

private:
    const std::uint32_t idx;
    const std::tuple<IRQn_Type> irqn;
    TIM_TypeDef* p_registers;
};

template<> class rcc<Basic_timer> : private cml::Non_constructible
{
public:
    template<std::uint32_t peripheral_base_address>
    static void enable(Handle<peripheral_base_address>, bool a_enable_in_lp)                             = delete;
    template<std::uint32_t peripheral_base_address> static void disable(Handle<peripheral_base_address>) = delete;
};

template<> void rcc<Basic_timer>::enable<TIM6_BASE>(Handle<TIM6_BASE>, bool a_enable_in_lp);
template<> void rcc<Basic_timer>::disable<TIM6_BASE>(Handle<TIM6_BASE>);
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
template<> void rcc<Basic_timer>::enable<TIM7_BASE>(Handle<TIM7_BASE>, bool a_enable_in_lp);
template<> void rcc<Basic_timer>::disable<TIM7_BASE>(Handle<TIM7_BASE>);
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc
