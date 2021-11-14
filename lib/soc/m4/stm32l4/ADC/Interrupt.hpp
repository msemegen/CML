#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <array>
#include <cstdint>

// soc
#include <soc/Factory.hpp>
#include <soc/m4/stm32l4/ADC/ADC.hpp>
#include <soc/m4/stm32l4/IRQ_config.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<ADC> : private cml::Non_copyable
{
public:
    enum class Mode : std::uint32_t
    {
        single     = 0x0u,
        continuous = ADC_CFGR_CONT,
        none
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

    struct Callback
    {
        using Function = void (*)(std::uint16_t a_value, bool a_series_end, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    ~Interrupt()
    {
        this->disable();
    }

    void disable();

    template<std::size_t length>
    void enable(const IRQ_config& a_irq_config, const std::array<Channel, length>& a_channels)
    {
        this->enable(a_irq_config, a_channels.data(), a_channels.size());
    }

    void register_callack(Mode a_mode, const Callback& a_callback);

    ADC* get_handle()
    {
        return this->p_adc;
    }

    const ADC* get_handle() const
    {
        return this->p_adc;
    }

private:
    Interrupt(ADC* a_p_adc, IRQn_Type a_irqn)
        : p_adc(a_p_adc)
        , irqn(a_irqn)
    {
    }

    void enable(const IRQ_config& a_irq_config, const Channel* a_p_channels, std::size_t a_channels_count);

    void set_irq_context();
    void clear_irq_context();

    ADC* p_adc;
    IRQn_Type irqn;

    Callback callback;

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
    friend void ADC_interrupt_handler(Interrupt<ADC>* a_p_this);
};
} // namespace stm32l4
} // namespace m4
} // namespace soc