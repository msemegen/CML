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
#include <soc/m4/stm32l4/ADC/DMA.hpp>
#include <soc/m4/stm32l4/defs.hpp>

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
#if defined(SOC_PLLSAI_PRESENT)
        PLLSAI1
#endif
    };

    enum class PCLK_prescaler : std::uint32_t
    {
        _1 = ADC_CCR_CKMODE_0,
        _2 = ADC_CCR_CKMODE_1,
        _4 = ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1
    };

#if defined(SOC_PLLSAI_PRESENT)
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
#endif

    template<Clock_source source> static void enable(PCLK_prescaler a_prescaler, bool a_enable_in_lp)
    {
        static_assert(Clock_source::PCLK == source);
    }

#if defined(SOC_PLLSAI_PRESENT)
    template<Clock_source source> static void enable(PLLSAI1_prescaler a_prescaler, bool a_enable_in_lp)
    {
        static_assert(Clock_source::PLLSAI1 == source);
    }
#endif

    static void disable();
};

template<>
void rcc<ADC>::enable<rcc<ADC>::Clock_source::PCLK>(rcc<ADC>::PCLK_prescaler a_prescaler, bool a_enable_in_lp);

#if defined(SOC_PLLSAI_PRESENT)
template<>
void rcc<ADC>::enable<rcc<ADC>::Clock_source::PLLSAI1>(rcc<ADC>::PLLSAI1_prescaler a_prescaler, bool a_enable_in_lp);
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
#if defined(SOC_ADC1_PRESENT)
template<> class Peripheral<m4::stm32l4::ADC, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::ADC create()
    {
        return m4::stm32l4::ADC(0U, ADC1, IRQn_Type::ADC1_IRQn);
    }
};

template<> class Peripheral<m4::stm32l4::ADC, 1, m4::stm32l4::DMA<>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::ADC> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::ADC>(
            0u, ADC1, DMA1_CSELR, DMA1_Channel1, DMA1_Channel1_IRQn, { 0x0u, 0xFu });
    }
};

template<> class Peripheral<m4::stm32l4::ADC, 1, m4::stm32l4::DMA<>, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::ADC> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::ADC>(
            1u, ADC1, DMA2_CSELR, DMA1_Channel3, DMA2_Channel3_IRQn, { 0x0u, 0xFu });
    }
};
#endif

#if defined(SOC_ADC2_PRESENT)
template<> class Peripheral<m4::stm32l4::ADC, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::ADC create()
    {
        return m4::stm32l4::ADC(1, ADC2, IRQn_Type::ADC1_2_IRQn);
    }
};

template<> class Peripheral<m4::stm32l4::ADC, 2, m4::stm32l4::DMA<>, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::ADC> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::ADC>(
            0u, ADC2, DMA1_CSELR, DMA1_Channel2, DMA1_Channel2_IRQn, { 0x0u, 0xFu });
    }
};

template<> class Peripheral<m4::stm32l4::ADC, 1, m4::stm32l4::DMA<>, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::DMA<m4::stm32l4::ADC> create()
    {
        return m4::stm32l4::DMA<m4::stm32l4::ADC>(
            1u, ADC2, DMA2_CSELR, DMA2_Channel4, DMA2_Channel4_IRQn, { 0x0u, 0xFu });
    }
};
#endif
} // namespace soc