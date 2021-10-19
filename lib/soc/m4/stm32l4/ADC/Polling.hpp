#pragma once

/*
 *   Name: Polling.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <array>

// soc
#include <soc/m4/stm32l4/ADC/ADC.hpp>
#include <soc/m4/stm32l4/Polling.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Polling<ADC> : private cml::Non_copyable
{
public:
    enum class Mode : std::uint32_t
    {
        single     = 0x0u,
        continuous = ADC_CFGR_CONT
    };

    struct Channel
    {
        enum class Id : std::uint32_t
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

        enum class Sampling_time : std::uint32_t
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

public:
    Polling(ADC* a_p_adc)
        : p_adc(a_p_adc)
    {
    }
    ~Polling()
    {
        this->disable();
    }

    void disable();

    template<std::size_t length> void enable(const std::array<Channel, length>& a_channels)
    {
        this->enable(a_channels.data(), a_channels.size());
    }

    template<std::size_t length> void read(Mode a_mode, std::array<uint16_t, length>* a_p_buffer)
    {
        this->read(a_mode, a_p_buffer->data(), a_p_buffer->size());
    }
    template<std::size_t length> bool read(Mode a_mode, std::array<uint16_t, length>* a_p_buffer, uint32_t a_timeout)
    {
        return this->read(a_mode, a_p_buffer->data(), a_p_buffer->size(), a_timeout);
    }

    ADC* get_handle()
    {
        return this->p_adc;
    }

    const ADC* get_handle() const
    {
        return this->p_adc;
    }

private:
    void enable(const Channel* a_p_channels, std::size_t a_channels_count);

    void read(Mode a_mode, uint16_t* a_p_buffer, std::size_t a_buffer_capacity);
    bool read(Mode a_mode, uint16_t* a_p_buffer, std::size_t a_buffer_capacity, uint32_t a_timeout);

private:
    ADC* p_adc;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc