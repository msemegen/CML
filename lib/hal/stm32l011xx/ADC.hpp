#pragma once

/*
    Name: ADC.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l011xx.h>

//cml
#include <common/integer.hpp>
#include <common/macros.hpp>
#include <common/time_tick.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

class ADC
{
public:

    enum class Id
    {
        _1 = 0
    };

    enum class Resolution : common::uint32
    {
        _6_bit  = ADC_CFGR1_RES_1 | ADC_CFGR1_RES_0,
        _8_bit  = ADC_CFGR1_RES_1,
        _10_bit = ADC_CFGR1_RES_0,
        _12_bit = 0u,
    };

    enum class Channel : common::uint32
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
        temperature_sensor,
        voltage_reference
    };

    enum class Sampling_time : common::uint32
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
        enum class Divider : common::uint32
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
        enum class Divider : common::uint32
        {
            _1 = ADC_CFGR2_CKMODE_0 | ADC_CFGR2_CKMODE_1,
            _2 = ADC_CFGR2_CKMODE_0,
            _4 = ADC_CFGR2_CKMODE_1,
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

public:

    ADC(Id a_id)
    {
        unused(a_id);
    }

    ~ADC()
    {
        this->disable();
    }

    ADC()           = delete;
    ADC(const ADC&) = default;
    ADC(ADC&&)      = default;

    ADC& operator = (const ADC&) = default;
    ADC& operator = (ADC&&)      = default;

    bool enable(Resolution a_resolution, const Synchronous_clock& a_clock, common::time_tick a_timeout);
    bool enable(Resolution a_resolution, const Asynchronous_clock& a_clock, common::time_tick a_timeout);

    void disable();

    void set_active_channels(Sampling_time a_sampling_time, const Channel* a_p_channels);
    void clear_active_channels();

    void read_polling(common::uint16* a_p_data, common::uint32 a_count);
    bool read_polling(common::uint16* a_p_data, common::uint32 a_count, common::time_tick a_timeout);

private:

    bool enable(Resolution a_resolution, common::time_tick a_start, common::time_tick a_timeout);

};

} // namespace cml
} // namespace hal
} // namespace stm32l011xx