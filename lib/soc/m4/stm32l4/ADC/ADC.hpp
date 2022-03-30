#pragma once

/*
 *   Name: ADC.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <array>
#include <cstdint>
#include <limits>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Peripheral.hpp>
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/defs.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Duration.hpp>
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>
#include <cml/various.hpp>

#if defined(SOC_ADC1_PRESENT) && !defined(SOC_ADC2_PRESENT)
#define ADC_COMMON_T ADC1_COMMON
#endif

#if defined(SOC_ADC1_PRESENT) && defined(SOC_ADC2_PRESENT)
#define ADC_COMMON_T ADC12_COMMON
#endif

namespace soc {
namespace m4 {
namespace stm32l4 {
class ADC
{
public:
    enum class Mode : std::uint32_t
    {
        single        = 0x0u,
        continuous    = ADC_CFGR_CONT,
        discontinuous = ADC_CFGR_DISCEN
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
    struct Channel
    {
        enum class Id : std::uint32_t
        {
            voltage_reference,
#if defined(SOC_ADC_CHANNELS_5_6_7_8_9_10_11_12_15_16)
            _5  = 5,
            _6  = 6,
            _7  = 7,
            _8  = 8,
            _9  = 9,
            _10 = 10,
            _11 = 11,
            _12 = 12,
            _15 = 15,
            _16 = 16,
#endif
#if defined(SOC_ADC_CHANNELS_1_2_3_4_5_6_7_8_9_10_11_12_13_14_15_16)
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
#endif
#if defined(SOC_ADC_CHANNELS_1_2_3_4_5_6_7_8_9_10_11_12_13_15_16)
            _1  = 1,
            _2  = 2,
            _3  = 3,
            _4  = 4,
            _5  = 5,
            _6  = 6,
            _7  = 7,
            _8  = 8,
            _9  = 9,
            _10 = 10,
            _11 = 11,
            _12 = 12,
            _13 = 13,
            _15 = 15,
            _16 = 16,
#endif
#if defined(SOC_ADC_CHANNELS_4_5_6_7_8_9_10_11_12_15_16)
            _4  = 4,
            _5  = 5,
            _6  = 6,
            _7  = 7,
            _8  = 8,
            _9  = 9,
            _10 = 10,
            _11 = 11,
            _12 = 12,
            _15 = 15,
            _16 = 16,
#endif
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

    class Polling
    {
    public:
        template<Mode mode>
        void read(uint16_t* a_p_buffer, std::size_t a_buffer_capacity, std::size_t a_group_size) = delete;
        template<Mode mode> bool read(uint16_t* a_p_buffer,
                                      std::size_t a_buffer_capacity,
                                      std::size_t a_group_size,
                                      cml::Milliseconds a_timeout)                               = delete;

        template<Mode mode> void read(uint16_t* a_p_buffer, std::size_t a_buffer_capacity) = delete;
        template<Mode mode>
        bool read(uint16_t* a_p_buffer, std::size_t a_buffer_capacity, cml::Milliseconds a_timeout) = delete;

    private:
        ADC* p_ADC;
        friend ADC;
    };
    class Interrupt
    {
    public:
        struct Read_callback
        {
            using Function = void (*)(std::uint16_t a_value, bool a_series_end, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void enable(const IRQ_config& a_irq_config);
        void disable();

        template<Mode mode> void read_start(const Read_callback& a_callback)                           = delete;
        template<Mode mode> void read_start(const Read_callback& a_callback, std::size_t a_group_size) = delete;
        void read_stop();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(this->p_ADC->irqn);
        }

    private:
        void set_irq_context();
        void clear_irq_context();

        ADC* p_ADC;
        friend ADC;
    };
    template<std::size_t length>
    bool enable(Resolution a_resolution, const std::array<Channel, length>& a_channels, cml::Milliseconds a_timeout)
    {
        return this->enable(a_resolution, a_channels.data(), a_channels.size(), a_timeout);
    }
    void disable();

    constexpr Calibration_data get_calibration_data() const
    {
        return { *(reinterpret_cast<const std::uint16_t*>(0x1FFF75A8)),
                 *(reinterpret_cast<const std::uint16_t*>(0x1FFF75CA)),
                 *(reinterpret_cast<const std::uint16_t*>(0x1FFF75AA)) };
    }

    std::uint32_t get_idx() const
    {
        return this->idx;
    }

    bool is_enabled() const
    {
        return ADC_CR_ADEN == cml::bit_flag::get(this->p_registers->CR, ADC_CR_ADEN | ADC_CR_ADDIS);
    }

    bool is_created() const
    {
        return std::numeric_limits<decltype(this->idx)>::max() != this->idx && nullptr != this->p_registers;
    }

    operator ADC_TypeDef*()
    {
        return this->p_registers;
    }

    operator const ADC_TypeDef*() const
    {
        return this->p_registers;
    }

    Polling polling;
    Interrupt interrupt;

private:
    ADC(std::size_t a_idx, ADC_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
    {
        this->polling.p_ADC   = this;
        this->interrupt.p_ADC = this;
    }

    bool enable(ADC::Resolution a_resolution,
                const ADC::Channel* a_p_channels,
                std::size_t a_channels_count,
                cml::Milliseconds a_timeout);

    std::uint32_t idx;
    ADC_TypeDef* p_registers;

    IRQn_Type irqn;
    Interrupt::Read_callback read_callback;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
    friend void ADC_interrupt_handler(ADC* a_p_this);
};

template<> void ADC::Polling::read<ADC::Mode::single>(uint16_t* a_p_buffer, std::size_t a_buffer_capacity);
template<> void ADC::Polling::read<ADC::Mode::continuous>(uint16_t* a_p_buffer, std::size_t a_buffer_capacity);
template<> void ADC::Polling::read<ADC::Mode::discontinuous>(uint16_t* a_p_buffer,
                                                             std::size_t a_buffer_capacity,
                                                             std::size_t a_group_size);

template<> bool
ADC::Polling::read<ADC::Mode::single>(uint16_t* a_p_buffer, std::size_t a_buffer_capacity, cml::Milliseconds a_timeout);
template<> bool ADC::Polling::read<ADC::Mode::continuous>(uint16_t* a_p_buffer,
                                                          std::size_t a_buffer_capacity,
                                                          cml::Milliseconds a_timeout);
template<> bool ADC::Polling::read<ADC::Mode::discontinuous>(uint16_t* a_p_buffer,
                                                             std::size_t a_buffer_capacity,
                                                             std::size_t a_group_size,
                                                             cml::Milliseconds a_timeout);

template<> void ADC::Interrupt::read_start<ADC::Mode::single>(const Read_callback& a_callback);
template<> void ADC::Interrupt::read_start<ADC::Mode::continuous>(const Read_callback& a_callback);
template<>
void ADC::Interrupt::read_start<ADC::Mode::discontinuous>(const Read_callback& a_callback, std::size_t a_group_size);
} // namespace stm32l4
} // namespace m4
} // namespace soc