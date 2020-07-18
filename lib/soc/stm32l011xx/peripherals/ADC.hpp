#pragma once

/*
    Name: ADC.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

//externals
#include <stm32l011xx.h>

//cml
#include <cml/Non_copyable.hpp>
#include <cml/time.hpp>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

class ADC : private cml::Non_copyable
{
public:

    enum class Id
    {
        _1 = 0
    };

    enum class Resolution : uint32_t
    {
        _6_bit  = ADC_CFGR1_RES_1 | ADC_CFGR1_RES_0,
        _8_bit  = ADC_CFGR1_RES_1,
        _10_bit = ADC_CFGR1_RES_0,
        _12_bit = 0u,
    };

    enum class Channel : uint32_t
    {
        _0,
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
        voltage_reference,
        temperature_sensor
    };

    enum class Sampling_time : uint32_t
    {
        _1_5_clock_cycles   = 0x0u,
        _3_5_clock_cycles   = ADC_SMPR_SMP_0,
        _7_5_clock_cycles   = ADC_SMPR_SMP_1,
        _12_5_clock_cycles  = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1,
        _19_5_clock_cycles  = ADC_SMPR_SMP_2,
        _39_5_clock_cycles  = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_2,
        _79_5_clock_cycles  = ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2,
        _160_5_clock_cycles = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2
    };

    struct Asynchronous_clock
    {
        enum class Divider : uint32_t
        {
            _1   = 0x0u,
            _2   = ADC_CCR_PRESC_0,
            _4   = ADC_CCR_PRESC_1,
            _6   = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1,
            _8   = ADC_CCR_PRESC_2,
            _10  = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_2,
            _12  = ADC_CCR_PRESC_1 | ADC_CCR_PRESC_2,
            _16  = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_2,
            _32  = ADC_CCR_PRESC_3,
            _64  = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_3,
            _128 = ADC_CCR_PRESC_1 | ADC_CCR_PRESC_3,
            _256 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_3,
            unknown
        };

        enum class Source
        {
            hsi,
            unknown
        };

        Source source   = Source::unknown;
        Divider divider = Divider::unknown;
    };

    struct Synchronous_clock
    {
        enum class Divider : uint32_t
        {
            _1 = ADC_CFGR2_CKMODE_0 | ADC_CFGR2_CKMODE_1,
            _2 = ADC_CFGR2_CKMODE_0,
            _4 = ADC_CFGR2_CKMODE_1,
            unknown
        };

        enum class Source
        {
            plck,
            unknown
        };

        Source source   = Source::unknown;
        Divider divider = Divider::unknown;
    };

    struct Calibration_data
    {
        uint16_t temperature_sensor_data_1  = 0;
        uint16_t temperature_sensor_data_2  = 0;
        uint16_t internal_voltage_reference = 0;
    };

    struct Conversion_callback
    {
        using Function = bool(*)(uint16_t a_value, bool a_series_end, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:

    ADC(Id){}

    ~ADC()
    {
        this->disable();
    }

    bool enable(Resolution a_resolution,
                const Synchronous_clock& a_clock,
                uint32_t a_irq_priority,
                cml::time::tick a_timeout);

    bool enable(Resolution a_resolution,
                const Asynchronous_clock& a_clock,
                uint32_t a_irq_priority,
                cml::time::tick a_timeout);

    void disable();

    void set_active_channels(Sampling_time a_sampling_time, const Channel* a_p_channels, uint32_t a_channels_count);
    void clear_active_channels();

    void read_polling(uint16_t* a_p_data, uint32_t a_count);
    bool read_polling(uint16_t* a_p_data, uint32_t a_count, cml::time::tick a_timeout);

    void start_read_it(const Conversion_callback& a_callback);
    void stop_read_it();

    void set_resolution(Resolution a_resolution);

    uint32_t get_active_channels_count() const;

    constexpr Calibration_data get_calibration_data() const
    {
        return { *(reinterpret_cast<const uint16_t*>(0x1FF8007A)),
                 *(reinterpret_cast<const uint16_t*>(0x1FF8007E)),
                 *(reinterpret_cast<const uint16_t*>(0x1FF80078))
        };
    }

    constexpr Id get_id() const
    {
        return Id::_1;
    }

private:

    bool enable(Resolution a_resolution,
                cml::time::tick a_start,
                uint32_t a_irq_priority,
                cml::time::tick a_timeout);

private:

    friend void adc_interrupt_handler(ADC* a_p_this);
};

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc