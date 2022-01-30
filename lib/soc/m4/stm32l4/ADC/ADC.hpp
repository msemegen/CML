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
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class ADC : private cml::Non_copyable
{
public:
    enum class Resolution : std::uint32_t
    {
        _6_bit  = ADC_CFGR_RES_1 | ADC_CFGR_RES_0,
        _8_bit  = ADC_CFGR_RES_1,
        _10_bit = ADC_CFGR_RES_0,
        _12_bit = 0u,
    };

    enum class Mode : std::uint32_t
    {
        single     = 0x0u,
        continuous = ADC_CFGR_CONT,
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
#if defined(STM32L431CB) || defined(STM32L431CC) || defined(STM32L433CB) || defined(STM32L433CC) || \
    defined(STM32L443CC) || defined(STM32L433RC) || defined(STM32L452RE) || defined(STM32L412RB) || \
    defined(STM32L433RB) || defined(STM32L433VC) || defined(STM32L443RC) || defined(STM32L443VC) || \
    defined(STM32L451RC) || defined(STM32L451RE) || defined(STM32L451VC) || defined(STM32L451VE) || \
    defined(STM32L452RC) || defined(STM32L452VC) || defined(STM32L452VE) || defined(STM32L462RE) || \
    defined(STM32L462VE) || defined(STM32L412R8) || defined(STM32L422RB)
            _11,
            _12,
            _13,
            _14,
#if defined(STM32L433RC) || defined(STM32L452RE) || defined(STM32L412RB) || defined(STM32L433RB) || \
    defined(STM32L433VC) || defined(STM32L443RC) || defined(STM32L443VC) || defined(STM32L451RC) || \
    defined(STM32L451RE) || defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452RC) || \
    defined(STM32L452VC) || defined(STM32L452VE) || defined(STM32L462RE) || defined(STM32L462VE) || \
    defined(STM32L412R8) || defined(STM32L422RB)
            _15,
#if defined(STM32L433RB) || defined(STM32L433VC) || defined(STM32L443RC) || defined(STM32L443VC) || \
    defined(STM32L451RC) || defined(STM32L451RE) || defined(STM32L451VC) || defined(STM32L451VE) || \
    defined(STM32L452RC) || defined(STM32L452VC) || defined(STM32L452VE) || defined(STM32L462RE) || \
    defined(STM32L462VE) || defined(STM32L412R8) || defined(STM32L422RB) || defined(STM32L452RE)
            _16,
#endif
#endif
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

    class Polling : private cml::Non_copyable
    {
    public:
        enum class Mode : std::uint32_t
        {
            single     = 0x0u,
            continuous = ADC_CFGR_CONT
        };

        Polling(Polling&&) = default;
        Polling& operator=(Polling&&) = default;

        ~Polling()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void disable();
        void read(Mode a_mode, uint16_t* a_p_buffer, std::size_t a_buffer_capacity);
        bool read(Mode a_mode, uint16_t* a_p_buffer, std::size_t a_buffer_capacity, uint32_t a_timeout);

        template<std::size_t length> void enable(const std::array<Channel, length>& a_channels)
        {
            this->enable(a_channels.data(), a_channels.size());
        }

        template<std::size_t length> void read(Mode a_mode, std::array<uint16_t, length>* a_p_buffer)
        {
            this->read(a_mode, a_p_buffer->data(), a_p_buffer->size());
        }
        template<std::size_t length>
        bool read(Mode a_mode, std::array<uint16_t, length>* a_p_buffer, uint32_t a_timeout)
        {
            return this->read(a_mode, a_p_buffer->data(), a_p_buffer->size(), a_timeout);
        }

        bool is_enabled()
        {
            return this->enabled;
        }

        bool is_created()
        {
            return nullptr != this->p_ADC;
        }

    private:
        Polling()
            : p_ADC(nullptr)
            , enabled(false)
        {
        }

        Polling(ADC* a_p_ADC)
            : p_ADC(a_p_ADC)
            , enabled(false)
        {
        }

        void enable(const Channel* a_p_channels, std::size_t a_channels_count);

        ADC* p_ADC;
        bool enabled;

        friend ADC;
    };
    class Interrupt : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(std::uint16_t a_value, bool a_series_end, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        Interrupt(Interrupt&&) = default;
        Interrupt& operator=(Interrupt&&) = default;

        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void disable();

        template<std::size_t length>
        void enable(const IRQ_config& a_irq_config, const std::array<Channel, length>& a_channels)
        {
            this->enable(a_irq_config, a_channels.data(), a_channels.size());
        }

        void register_callack(Mode a_mode, const Callback& a_callback);
        void unregister_callback();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(this->irqn);
        }

        bool is_created() const
        {
            return nullptr != this->p_ADC && static_cast<IRQn_Type>(std::numeric_limits<int32_t>::max()) != this->irqn;
        }

    private:
        Interrupt()
            : p_ADC(nullptr)
            , irqn(static_cast<IRQn_Type>(std::numeric_limits<int32_t>::max()))
        {
        }

        Interrupt(ADC* a_p_ADC, IRQn_Type a_irqn)
            : p_ADC(a_p_ADC)
            , irqn(a_irqn)
        {
        }

        void enable(const IRQ_config& a_irq_config, const Channel* a_p_channels, std::size_t a_channels_count);

        void set_irq_context();
        void clear_irq_context();

        ADC* p_ADC;
        IRQn_Type irqn;

        Callback callback;

        friend ADC;
        friend void ADC_interrupt_handler(ADC* a_p_this);
    };

    ADC(ADC&&)   = default;
    ADC& operator=(ADC&&) = default;

    ADC()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
    {
    }
    ~ADC()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    bool enable(Resolution a_resolution, std::uint32_t a_timeout_ms);
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
        : polling(this)
        , interrupt(this, a_irqn)
        , idx(a_idx)
        , p_registers(a_p_registers)
    {
    }

    const std::uint32_t idx;
    ADC_TypeDef* p_registers;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

void ADC_interrupt_handler(ADC* a_p_this);

} // namespace stm32l4
} // namespace m4
} // namespace soc