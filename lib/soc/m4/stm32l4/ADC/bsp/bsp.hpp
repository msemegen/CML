#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/ADC/ADC.hpp>
#include <soc/m4/stm32l4/ADC/Interrupt.hpp>
#include <soc/m4/stm32l4/ADC/Polling.hpp>
#include <soc/m4/stm32l4/Factory.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class rcc<ADC> : private cml::Non_constructible
{
public:
    enum class Clock_source : std::uint32_t
    {
        PCLK,
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        PLLSAI1
#endif
    };

    enum class PCLK_prescaler : std::uint32_t
    {
        _1 = ADC_CCR_CKMODE_0,
        _2 = ADC_CCR_CKMODE_1,
        _4 = ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1
    };

    enum class PLLSAI1_prescaler : std::uint32_t
    {
        _1 = 0x0u,
        _2,
        _4,
        _6,
        _8,
        _10,
        _12,
        _16,
        _32,
        _64,
        _128,
        _256
    };

    template<Clock_source source> static void enable(PCLK_prescaler a_prescaler, bool a_enable_in_lp)
    {
        static_assert(Clock_source::PCLK == source);
    }

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    template<Clock_source source> static void enable(PLLSAI1_prescaler a_prescaler, bool a_enable_in_lp)
    {
        static_assert(Clock_source::PLLSAI1 == source);
    }
#endif

    static void disable();
};

template<> class Factory<ADC, 1u> : private cml::Non_constructible
{
public:
    static ADC create()
    {
        return ADC(0, ADC1);
    }
};
template<> class Factory<Polling<ADC>, 1u> : private cml::Non_constructible
{
public:
    static Polling<ADC> create(ADC* a_p_ADC)
    {
        return Polling<ADC>(a_p_ADC);
    }
};
template<> class Factory<Interrupt<ADC>, 1u> : private cml::Non_constructible
{
public:
    static Interrupt<ADC> create(ADC* a_p_ADC)
    {
        return Interrupt<ADC>(a_p_ADC, IRQn_Type::ADC1_IRQn);
    }
};

template<>
void rcc<ADC>::enable<rcc<ADC>::Clock_source::PCLK>(rcc<ADC>::PCLK_prescaler a_prescaler, bool a_enable_in_lp);

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<>
void rcc<ADC>::enable<rcc<ADC>::Clock_source::PLLSAI1>(rcc<ADC>::PLLSAI1_prescaler a_prescaler, bool a_enable_in_lp);
#endif

#if defined(STM32L412xx) || defined(STM32L422xx)
template<> class Factory<ADC, 2u> : private cml::Non_constructible
{
public:
    static ADC create()
    {
        return ADC(1, ADC2);
    }
};
template<> class Factory<Polling<ADC>, 2u> : private cml::Non_constructible
{
public:
    static Polling<ADC> create(ADC* a_p_ADC)
    {
        return Polling<ADC>(a_p_ADC);
    }
};
template<> class Factory<Interrupt<ADC>, 2u> : private cml::Non_constructible
{
public:
    static Interrupt<ADC> create(ADC* a_p_ADC)
    {
        return Interrupt<ADC>(a_p_ADC, IRQn_Type::ADC1_2_IRQn);
    }
};
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc