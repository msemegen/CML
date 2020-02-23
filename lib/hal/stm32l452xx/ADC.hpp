#pragma once

/*
    Name: USART.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l452xx.h>

//cml
#include <common/integer.hpp>

namespace cml {
namespace hal {
namespace stm32l4xx {

class ADC
{
public:

    enum class Id : common::uint32
    {
        _1 = 0u
    };

    struct Config
    {
        enum class Resolution : common::uint32
        {
            _6_bit = ADC_CFGR_RES_1 | ADC_CFGR_RES_0,
            _8_bit = ADC_CFGR_RES_1,
            _10_bit = ADC_CFGR_RES_0,
            _12_bit = 0u,
        };

    };

    struct Channel
    {
        enum class Id : common::uint32
        {
            unknown
        };

        enum class Sampling_time : common::uint32
        {
            unknown
        };

        Id id                       = Id::unknown;
        Sampling_time sampling_time = Sampling_time::unknown;
    };


public:

    void enable(const Config& a_config);
    void disable();

    void set_channels(const Channel* a_p_channels, common::uint32 a_channels_count);
    void clear_channels();
};

} // namespace stm32l4xx
} // namespace hal
} // namespace cml