#pragma once

/*
 *   Name: USART.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// std
#include <cstddef>
#include <cstdint>

// soc
#include <soc/Peripheral.hpp>
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/DMA.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Duration.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class USART : private cml::Non_copyable
{
public:
    template<typename Config_t> Config_t get_config() const = delete;

    enum class Event_flag : std::uint32_t
    {
        none              = 0x0u,
        framing_error     = 0x1u,
        parity_error      = 0x2u,
        overrun           = 0x4u,
        noise_detected    = 0x8u,
        idle              = 0x10u,
        transfer_complete = 0x20u
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
        enum class Flow_control_flag : std::uint32_t
        {
            none            = 0x0u,
            request_to_send = USART_CR3_RTSE,
            clear_to_send   = USART_CR3_CTSE,
        };
        enum Sampling_method : std::uint32_t
        {
            three_sample_bit = 0,
            one_sample_bit   = USART_CR3_ONEBIT,
        };
        enum class Mode_flag : std::uint32_t
        {
            tx = USART_CR1_TE,
            rx = USART_CR1_RE,
        };

        std::uint32_t baud_rate         = 0;
        std::uint32_t clock_freq_Hz     = 0;
        Oversampling oversampling       = cml::various::get_enum_incorrect_value<Oversampling>();
        Stop_bits stop_bits             = cml::various::get_enum_incorrect_value<Stop_bits>();
        Flow_control_flag flow_control  = cml::various::get_enum_incorrect_value<Flow_control_flag>();
        Sampling_method sampling_method = cml::various::get_enum_incorrect_value<Sampling_method>();
        Mode_flag mode                  = cml::various::get_enum_incorrect_value<Mode_flag>();
    };
    struct Frame_config
    {
        enum class Word_length : uint32_t
        {
            _7_bit = USART_CR1_M1,
            _8_bit = 0x0u,
            _9_bit = USART_CR1_M0,
        };
        enum class Parity : uint32_t
        {
            none = 0x0u,
            even = USART_CR1_PCE,
            odd  = USART_CR1_PCE | USART_CR1_PS,
        };

        Word_length word_length = cml::various::get_enum_incorrect_value<Word_length>();
        Parity parity           = cml::various::get_enum_incorrect_value<Parity>();
    };

    class Polling
    {
    public:
        struct Result
        {
            Event_flag event_flag            = cml::various::get_enum_incorrect_value<Event_flag>();
            std::size_t data_length_in_words = 0;
        };

        Result transmit(const void* a_p_data, std::size_t a_data_size_in_words);
        Result transmit(const void* a_p_data, std::size_t a_data_size_in_words, cml::Milliseconds a_timeout);

        Result receive(void* a_p_data, std::size_t a_data_size_in_words);
        Result receive(void* a_p_data, std::size_t a_data_size_in_words, cml::Milliseconds a_timeout);

    private:
        USART* p_USART = nullptr;
        friend class USART;
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

        void transmit_start(const Transmit_callback& a_callback);
        void transmit_stop();

        void receive_start(const Receive_callback& a_callback);
        void receive_stop();

        void event_listening_start(const Event_callback& a_callback, Event_flag a_enabled_events);
        void event_listening_stop();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(this->p_USART->irqn);
        }

    private:
        void set_irq_context();
        void clear_irq_context();

        USART* p_USART = nullptr;
        friend class USART;
    };

    USART(USART&&) = default;
    USART& operator=(USART&&) = default;

    USART()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
        , enabled_interrupt_events(Event_flag::none)
    {
    }
    ~USART()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    bool enable(const Enable_config& a_config, const Frame_config& frame_config, cml::Milliseconds a_timeout);
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
    USART(std::size_t a_idx, USART_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
        , enabled_interrupt_events(Event_flag::none)
    {
        this->polling.p_USART   = this;
        this->interrupt.p_USART = this;
    }

    std::uint32_t idx;
    USART_TypeDef* p_registers;

    IRQn_Type irqn;
    Interrupt::Transmit_callback transmit_callback;
    Interrupt::Receive_callback receive_callback;
    Interrupt::Event_callback event_callback;
    Event_flag enabled_interrupt_events;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
    friend void USART_interrupt_handler(USART* a_p_this);
};

template<> inline USART::Enable_config USART::get_config<USART::Enable_config>() const
{
    return Enable_config {};
}
template<> inline USART::Frame_config USART::get_config<USART::Frame_config>() const
{
    return {
        static_cast<Frame_config::Word_length>(cml::bit_flag::get(this->p_registers->CR1, USART_CR1_M0 | USART_CR1_M1)),
        static_cast<Frame_config::Parity>(cml::bit_flag::get(this->p_registers->CR1, USART_CR1_PCE | USART_CR1_PS))
    };
}

constexpr USART::Enable_config::Flow_control_flag operator|(USART::Enable_config::Flow_control_flag a_f1,
                                                            USART::Enable_config::Flow_control_flag a_f2)
{
    return static_cast<USART::Enable_config::Flow_control_flag>(static_cast<uint32_t>(a_f1) |
                                                                static_cast<uint32_t>(a_f2));
}

constexpr USART::Enable_config::Flow_control_flag operator&(USART::Enable_config::Flow_control_flag a_f1,
                                                            USART::Enable_config::Flow_control_flag a_f2)
{
    return static_cast<USART::Enable_config::Flow_control_flag>(static_cast<uint32_t>(a_f1) &
                                                                static_cast<uint32_t>(a_f2));
}

constexpr USART::Enable_config::Flow_control_flag operator|=(USART::Enable_config::Flow_control_flag& a_f1,
                                                             USART::Enable_config::Flow_control_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr USART::Enable_config::Mode_flag operator|(USART::Enable_config::Mode_flag a_f1,
                                                    USART::Enable_config::Mode_flag a_f2)
{
    return static_cast<USART::Enable_config::Mode_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr USART::Enable_config::Mode_flag operator&(USART::Enable_config::Mode_flag a_f1,
                                                    USART::Enable_config::Mode_flag a_f2)
{
    return static_cast<USART::Enable_config::Mode_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr USART::Enable_config::Mode_flag operator|=(USART::Enable_config::Mode_flag& a_f1,
                                                     USART::Enable_config::Mode_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr USART::Event_flag operator|(USART::Event_flag a_f1, USART::Event_flag a_f2)
{
    return static_cast<USART::Event_flag>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}

constexpr USART::Event_flag operator&(USART::Event_flag a_f1, USART::Event_flag a_f2)
{
    return static_cast<USART::Event_flag>(static_cast<std::uint32_t>(a_f1) & static_cast<std::uint32_t>(a_f2));
}

constexpr USART::Event_flag operator|=(USART::Event_flag& a_f1, USART::Event_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

template<std::size_t id> class rcc<USART, id> : private cml::Non_constructible
{
public:
    enum class Clock_source : uint32_t
    {
        PCLK1,
        SYSCLK,
        HSI,
    };

    template<Clock_source> static void enable(bool a_enable_in_lp) = delete;
    static void disable()                                          = delete;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc