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
#include <soc/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
class ADC : private cml::Non_copyable
{
public:
    enum class Id : uint32_t
    {
        _1 = 0u,
#if defined(STM32L412xx) || defined(STM32L422xx)
        _2 = 1u
#endif
    };

    enum class Resolution : uint32_t
    {
        _6_bit  = ADC_CFGR_RES_1 | ADC_CFGR_RES_0,
        _8_bit  = ADC_CFGR_RES_1,
        _10_bit = ADC_CFGR_RES_0,
        _12_bit = 0u,
    };

    struct Channel
    {
        enum class Id : uint32_t
        {
            voltage_reference,
            _1,
            _2,
            _3,
            _4,
            _5,
            _6,
            _7,
            _8,
            _9,
            _10,
            _11,
            _12,
            _13,
            _14,
            _15,
            _16,
            temperature_sensor,
            battery_voltage,
        };

        enum class Sampling_time : uint32_t
        {
            _2_5_clock_cycles   = 0x0u,
            _6_5_clock_cycles   = 0x1u,
            _12_5_clock_cycles  = 0x2u,
            _24_5_clock_cycles  = 0x3u,
            _47_5_clock_cycles  = 0x4u,
            _92_5_clock_cycles  = 0x5u,
            _247_5_clock_cycles = 0x6u,
            _640_5_clock_cycles = 0x7u,
        };

        Id id                       = cml::various::get_enum_incorrect_value<Id>();
        Sampling_time sampling_time = cml::various::get_enum_incorrect_value<Sampling_time>();
    };

    struct Calibration_data
    {
        uint16_t temperature_sensor_data_1  = 0;
        uint16_t temperature_sensor_data_2  = 0;
        uint16_t internal_voltage_reference = 0;
    };

    struct Conversion_callback
    {
        using Function = void (*)(uint16_t a_value, bool a_series_end, ADC* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    ADC(Id a_id)
        : id(a_id)
    {
    }

    ~ADC()
    {
        this->disable();
    }

    bool enable(Resolution a_resolution, uint32_t a_irq_priority, uint32_t a_timeout);
    void disable();

    void set_active_channels(const Channel* a_p_channels, uint32_t a_channels_count);
    void clear_active_channels();

    void read_polling(uint16_t* a_p_data, uint32_t a_count);
    bool read_polling(uint16_t* a_p_data, uint32_t a_count, uint32_t a_timeout);

    void register_conversion_callback(const Conversion_callback& a_callback);
    void unregister_conversion_callback();

    uint32_t get_active_channels_count() const;

    void set_resolution(Resolution a_resolution);

    constexpr Calibration_data get_calibration_data() const
    {
        return { *(reinterpret_cast<const uint16_t*>(0x1FFF75A8)),
                 *(reinterpret_cast<const uint16_t*>(0x1FFF75CA)),
                 *(reinterpret_cast<const uint16_t*>(0x1FFF75AA)) };
    }

    constexpr Id get_id() const
    {
        return this->id;
    }

private:
    Conversion_callback callaback;

    Id id;

private:
    friend void adc_interrupt_handler(ADC* a_p_this);
};

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc

namespace soc {
namespace stm32l4 {
template<> struct rcc<peripherals::ADC>
{
    enum class Clock_source
    {
        pclk,
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        pllsai
#endif
    };

    enum class Prescaler : uint32_t
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

    static void enable(Clock_source a_source, Prescaler a_prescaler, bool a_enable_in_lp);
    static void disable();
};
} // namespace stm32l4
} // namespace soc
