#pragma once

/*
 *   Name: ADC.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Handle.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class ADC : private cml::Non_copyable
{
public:
    struct id : private cml::Non_constructible
    {
        constexpr static auto _1 = Handle<ADC1_BASE> {};
#if defined(STM32L412xx) || defined(STM32L422xx)
        constexpr static auto _2 = Handle<ADC2_BASE> {};
#endif
    };

    enum class Resolution : std::uint32_t
    {
        _6_bit  = ADC_CFGR_RES_1 | ADC_CFGR_RES_0,
        _8_bit  = ADC_CFGR_RES_1,
        _10_bit = ADC_CFGR_RES_0,
        _12_bit = 0u,
    };

    struct Calibration_data
    {
        std::uint16_t temperature_sensor_data_1  = 0u;
        std::uint16_t temperature_sensor_data_2  = 0u;
        std::uint16_t internal_voltage_reference = 0u;
    };

public:
    ADC(Handle<ADC1_BASE>)
        : idx(0u)
        , p_registers(ADC1)
    {
    }

#if defined(STM32L412xx) || defined(STM32L422xx)
    ADC(Handle<ADC2_BASE>)
        : idx(1u)
        , p_registers(ADC2)
    {
    }
#endif

    ~ADC();

    void enable(Resolution a_resolution);
    bool enable(Resolution a_resolution, std::uint32_t a_timeout);

    void disable();

    constexpr Calibration_data get_calibration_data() const
    {
        return { *(reinterpret_cast<const std::uint16_t*>(0x1FFF75A8)),
                 *(reinterpret_cast<const std::uint16_t*>(0x1FFF75CA)),
                 *(reinterpret_cast<const std::uint16_t*>(0x1FFF75AA)) };
    }

    std::uint32_t get_id() const
    {
        return this->idx;
    }

    operator ADC_TypeDef*()
    {
        return this->p_registers;
    }

    operator const ADC_TypeDef*() const
    {
        return this->p_registers;
    }

private:
    std::uint32_t idx;
    ADC_TypeDef* p_registers;
};

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

template<>
void rcc<ADC>::enable<rcc<ADC>::Clock_source::PCLK>(rcc<ADC>::PCLK_prescaler a_prescaler, bool a_enable_in_lp);

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<>
void rcc<ADC>::enable<rcc<ADC>::Clock_source::PLLSAI1>(rcc<ADC>::PLLSAI1_prescaler a_prescaler, bool a_enable_in_lp);
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc