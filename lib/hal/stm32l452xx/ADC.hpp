#pragma once

/*
    Name: ADC.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l452xx.h>

//cml
#include <common/integer.hpp>
#include <common/macros.hpp>
#include <common/Non_copyable.hpp>
#include <common/time_tick.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class ADC : private common::Non_copyable
{
public:

    using Conversion_callback = bool(*)(common::uint16 a_value, bool a_series_end, bool a_timeout);

    enum class Id : common::uint32
    {
        _1 = 0u
    };

    enum class Resolution : common::uint32
    {
        _6_bit  = ADC_CFGR_RES_1 | ADC_CFGR_RES_0,
        _8_bit  = ADC_CFGR_RES_1,
        _10_bit = ADC_CFGR_RES_0,
        _12_bit = 0u,
    };

    struct Channel
    {
        enum class Id : common::uint32
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
            unknown
        };

        enum class Sampling_time : common::uint32
        {
            _2_5_clock_cycles   = 0x0u,
            _6_5_clock_cycles   = 0x1u,
            _12_5_clock_cycles  = 0x2u,
            _24_5_clock_cycles  = 0x3u,
            _47_5_clock_cycles  = 0x4u,
            _92_5_clock_cycles  = 0x5u,
            _247_5_clock_cycles = 0x6u,
            _640_5_clock_cycles = 0x7u,
            unknown
        };

        Id id                       = Id::unknown;
        Sampling_time sampling_time = Sampling_time::unknown;
    };

    struct Synchronous_clock
    {
        enum class Divider : common::uint32
        {
            _1 = ADC_CCR_CKMODE_0,
            _2 = ADC_CCR_CKMODE_1,
            _4 = ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1,
            unknown
        };

        enum class Source
        {
            pclk,
            unknown
        };

        Source source   = Source::unknown;
        Divider divider = Divider::unknown;
    };

    struct Asynchronous_clock
    {
        enum class Divider : common::uint32
        {
            _1 = 0x0u,
            _2 = ADC_CCR_PRESC_0,
            _4 = ADC_CCR_PRESC_1,
            _6 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1,
            _8 = ADC_CCR_PRESC_2,
            _10 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_2,
            _12 = ADC_CCR_PRESC_1 | ADC_CCR_PRESC_2,
            _16 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_2,
            _32 = ADC_CCR_PRESC_3,
            _64 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_3,
            _128 = ADC_CCR_PRESC_1 | ADC_CCR_PRESC_3,
            _256 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_3,
            unknown
        };

        enum class Source
        {
            pllsai,
            unknown
        };

        Source source   = Source::unknown;
        Divider divider = Divider::unknown;
    };

    struct Calibration_data
    {
        common::uint16 temperature_sensor_data_1  = 0;
        common::uint16 temperature_sensor_data_2  = 0;
        common::uint16 internal_voltage_reference = 0;
    };

public:

    ADC(Id a_id)
    {
        unused(a_id);
    }

    ~ADC()
    {
        this->disable();
    }

    bool enable(Resolution a_resolution,
                const Asynchronous_clock& a_clock,
                common::uint32 a_irq_priority,
                common::time_tick a_timeout);

    bool enable(Resolution a_resolution,
                const Synchronous_clock& a_clock,
                common::uint32 a_irq_priority,
                common::time_tick a_timeout);

    void disable();

    void set_active_channels(const Channel* a_p_channels, common::uint32 a_channels_count);
    void clear_active_channels();

    void read_polling(common::uint16* a_p_data, common::uint32 a_count);
    bool read_polling(common::uint16* a_p_data, common::uint32 a_count, common::time_tick a_timeout);

    void start_read_it(Conversion_callback a_callback, common::time_tick a_timeout);

    void start_read_it(Conversion_callback a_callback)
    {
        this->start_read_it(a_callback, common::time_tick_infinity);
    }

    void stop_read_it();

    common::uint32 get_active_channels_count() const
    {
        return (ADC1->SQR1 & 0xFu) + 1;
    }

    void set_resolution(Resolution a_resolution);

    constexpr Calibration_data get_calibration_data() const
    {
        return { *(reinterpret_cast<const common::uint16*>(0x1FFF75A8)),
                 *(reinterpret_cast<const common::uint16*>(0x1FFF75CA)),
                 *(reinterpret_cast<const common::uint16*>(0x1FFF75AA))
        };
    }

    constexpr Id get_id() const
    {
        return Id::_1;
    }

private:

    struct IT_callback
    {
        Conversion_callback function = nullptr;

        common::time_tick start_timestamp = 0;
        common::time_tick timeout         = 0;
    };

private:

    bool enable(Resolution a_resolution,
                common::time_tick a_start,
                common::uint32 a_irq_priority,
                common::time_tick a_timeout);

private:

    IT_callback callaback;

private:

    friend void adc_handle_interrupt(ADC* a_p_this);
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml