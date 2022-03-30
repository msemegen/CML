#pragma once

/*
 *   Name: RS485.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>

// cml
#include <cml/Duration.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class RS485 : private cml::Non_copyable
{
public:
    enum class Event_flag : std::uint32_t
    {
        none              = 0x0u,
        framing_error     = 0x1u,
        parity_error      = 0x2u,
        overrun           = 0x4u,
        noise_detected    = 0x8u,
        idle              = 0x10u,
        transfer_complete = 0x20u,
        address_matched   = 0x40u,
    };

    struct Enable_config
    {
        enum class Oversampling : std::uint32_t
        {
            _8  = USART_CR1_OVER8,
            _16 = 0,
        };

        enum class Stop_bits : std::uint32_t
        {
            _0_5 = USART_CR2_STOP_0,
            _1   = 0x0u,
            _1_5 = USART_CR2_STOP_0 | USART_CR2_STOP_1,
            _2   = USART_CR2_STOP_1,
        };

        std::uint32_t baud_rate     = 0;
        std::uint32_t clock_freq_Hz = 0;
        Oversampling oversampling   = cml::various::get_enum_incorrect_value<Oversampling>();
        Stop_bits stop_bits         = cml::various::get_enum_incorrect_value<Stop_bits>();
        std::uint8_t address        = 0;
    };

    class Polling
    {
    public:
        struct Result
        {
            Event_flag bus_status            = cml::various::get_enum_incorrect_value<Event_flag>();
            std::size_t data_length_in_words = 0;
        };

        Result transmit(std::uint8_t a_address,
                        const void* a_p_data,
                        std::size_t a_data_size_in_words,
                        GPIO::Out::Pin* a_p_flow_control_pin);
        Result transmit(std::uint8_t a_address,
                        const void* a_p_data,
                        std::size_t a_data_size_in_words,
                        GPIO::Out::Pin* a_p_flow_control_pin,
                        cml::Milliseconds a_timeout);

        Result receive(void* a_p_data, std::size_t a_data_size_in_words, GPIO::Out::Pin* a_p_flow_control_pin);
        Result receive(void* a_p_data,
                       std::size_t a_data_size_in_words,
                       GPIO::Out::Pin* a_p_flow_control_pin,
                       cml::Milliseconds a_timeout);

    private:
        RS485* p_RS485;
        friend RS485;
    };
    class Interrupt
    {
    public:
        struct Transmit_callback
        {
            using Function = void (*)(volatile uint16_t* a_p_data, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };
        struct Receive_callback
        {
            using Function = void (*)(std::uint32_t a_data, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };
        struct Event_callback
        {
            using Function = void (*)(Event_flag a_event, void* a_p_user_data);

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

        void enable(const IRQ_config& a_config);
        void disable();

        void transmit_start(const Transmit_callback& a_callback, GPIO::Out::Pin* a_p_flow_control_pin);
        void transmit_stop();

        void receive_start(const Receive_callback& a_callback, GPIO::Out::Pin* a_p_flow_control_pin);
        void receive_stop();

        void register_Event_callback(const Event_callback& a_callback, Event_flag a_enabled_events);
        void unregister_Event_callback();

        bool is_enabled() const
        {
            return 0u != NVIC_GetEnableIRQ(this->p_RS485->irqn);
        }

    private:
        void set_irq_context();
        void clear_irq_context();

        RS485* p_RS485;
        friend class RS485;
    };

    RS485(RS485&&) = default;
    RS485& operator=(RS485&&) = default;

    RS485()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
        , enabled_interrupt_events(Event_flag::none)
    {
    }

    ~RS485()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    bool enable(const Enable_config& a_config, cml::Milliseconds a_timeout);
    void disable();

    std::uint32_t get_idx() const
    {
        return this->idx;
    }

    bool is_enabled() const
    {
        return cml::bit_flag::is(this->p_registers->CR1, USART_CR1_UE);
    }

    bool is_created() const
    {
        return std::numeric_limits<decltype(this->idx)>::max() != this->idx && nullptr != this->p_registers;
    }

    operator USART_TypeDef*()
    {
        return this->p_registers;
    }

    operator const USART_TypeDef*() const
    {
        return this->p_registers;
    }

    Polling polling;
    Interrupt interrupt;

private:
    RS485(std::size_t a_idx, USART_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
        , enabled_interrupt_events(Event_flag::none)
    {
        this->polling.p_RS485   = this;
        this->interrupt.p_RS485 = this;
    }

    std::uint32_t idx;
    USART_TypeDef* p_registers;

    IRQn_Type irqn;

    Interrupt::Transmit_callback transmit_callback;
    Interrupt::Receive_callback receive_callback;
    Interrupt::Event_callback event_callback;
    Event_flag enabled_interrupt_events;

    friend void RS485_interrupt_handler(RS485* a_p_this);
    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

void RS485_interrupt_handler(RS485* a_p_this);

constexpr RS485::Event_flag operator|(RS485::Event_flag a_f1, RS485::Event_flag a_f2)
{
    return static_cast<RS485::Event_flag>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}

constexpr RS485::Event_flag operator&(RS485::Event_flag a_f1, RS485::Event_flag a_f2)
{
    return static_cast<RS485::Event_flag>(static_cast<std::uint32_t>(a_f1) & static_cast<std::uint32_t>(a_f2));
}

constexpr RS485::Event_flag operator|=(RS485::Event_flag& a_f1, RS485::Event_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc