#pragma once

/*
 *   Name: USART.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <type_traits>

// externals
#include <stm32l0xx.h>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

class USART : private cml::Non_copyable
{
public:
    enum class Id : uint32_t
    {
        _2 = 0,
    };

    enum class Oversampling : uint32_t
    {
        _8  = USART_CR1_OVER8,
        _16 = 0,
        unknown
    };

    enum class Stop_bits : uint32_t
    {
        _0_5 = USART_CR2_STOP_0,
        _1   = 0x0u,
        _1_5 = USART_CR2_STOP_0 | USART_CR2_STOP_1,
        _2   = USART_CR2_STOP_1,
        unknown
    };

    enum class Flow_control_flag : uint32_t
    {
        none            = 0x0u,
        request_to_send = USART_CR3_RTSE,
        clear_to_send   = USART_CR3_CTSE,
        unknown
    };

    enum class Mode_flag : uint32_t
    {
        tx      = USART_CR1_TE,
        rx      = USART_CR1_RE,
        unknown = 0x0u
    };

    enum class Sampling_method : uint32_t
    {
        three_sample_bit = 0,
        one_sample_bit   = USART_CR3_ONEBIT,
        unknown
    };

    enum class Word_length : uint32_t
    {
        _7_bit = USART_CR1_M1,
        _8_bit = 0x0u,
        _9_bit = USART_CR1_M0,
        unknown
    };

    enum class Parity : uint32_t
    {
        none = 0x0u,
        even = USART_CR1_PCE,
        odd  = USART_CR1_PCE | USART_CR1_PS,
        unknown
    };

    enum class Bus_status_flag : uint32_t
    {
        ok             = 0x0,
        framing_error  = 0x1,
        parity_error   = 0x2,
        overrun        = 0x4,
        noise_detected = 0x8,
        unknown        = 0x10
    };

    struct Frame_format
    {
        Word_length word_length = Word_length::unknown;
        Parity parity           = Parity::unknown;
    };

    struct Config
    {
        uint32_t baud_rate              = 0;
        Oversampling oversampling       = Oversampling::unknown;
        Stop_bits stop_bits             = Stop_bits::unknown;
        Flow_control_flag flow_control  = Flow_control_flag::unknown;
        Sampling_method sampling_method = Sampling_method::unknown;
        Mode_flag mode                  = Mode_flag::unknown;
    };

    struct Clock
    {
        enum class Source : uint32_t
        {
            pclk,
            sysclk,
            hsi,
            unknown
        };

        Source source         = Source::unknown;
        uint32_t frequency_hz = 0;
    };

    struct Result
    {
        Bus_status_flag bus_status    = Bus_status_flag::unknown;
        uint32_t data_length_in_words = 0;
    };

    struct Transmit_callback
    {
        using Function = void (*)(volatile uint16_t* a_p_data,
                                  bool a_transfer_complete,
                                  USART* a_p_this,
                                  void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Receive_callback
    {
        using Function = void (*)(uint32_t a_data, bool a_idle, USART* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_status_callback
    {
        using Function = void (*)(Bus_status_flag a_bus_status, USART* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    USART(Id)
        : baud_rate(0)
    {
    }

    ~USART()
    {
        this->disable();
    }

    bool enable(const Config& a_config,
                const Frame_format& a_frame_format,
                const Clock& a_clock,
                uint32_t a_irq_priority,
                uint32_t a_timeout);

    void disable();

    template<typename Data_t> Result transmit_polling(const Data_t& a_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data));
    }

    template<typename Data_t> Result transmit_polling(const Data_t& a_data, uint32_t a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data), a_timeout);
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t));
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data, uint32_t a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t), a_timeout);
    }

    Result transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words);
    Result transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, uint32_t a_timeout);
    Result transmit_word(uint16_t a_word);
    Result transmit_word(uint16_t a_word, uint32_t a_timeout);

    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words);
    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, uint32_t a_timeout);

    void register_transmit_callback(const Transmit_callback& a_callback);
    void register_receive_callback(const Receive_callback& a_callback);
    void register_bus_status_callback(const Bus_status_callback& a_callback);

    void unregister_transmit_callback();
    void unregister_receive_callback();
    void unregister_bus_status_callback();

    void set_baud_rate(uint32_t a_baud_rate);
    void set_oversampling(Oversampling a_oversampling);
    void set_stop_bits(Stop_bits a_stop_bits);
    void set_flow_control(Flow_control_flag a_flow_control);
    void set_sampling_method(Sampling_method a_sampling_method);
    void set_frame_format(const Frame_format& a_frame_format);
    bool set_mode(Mode_flag a_mode, uint32_t a_timeout);

    bool is_transmit_callback() const
    {
        return nullptr != this->transmit_callback.function;
    }

    bool is_receive_callback() const
    {
        return nullptr != this->receive_callback.function;
    }

    bool is_bus_status_callback() const
    {
        return nullptr != this->bus_status_callback.function;
    }

    Oversampling get_oversampling() const;
    Stop_bits get_stop_bits() const;
    Flow_control_flag get_flow_control() const;
    Sampling_method get_sampling_method() const;
    Mode_flag get_mode() const;

    bool is_enabled() const;

    uint32_t get_baud_rate() const
    {
        return this->baud_rate;
    }

    const Clock& get_clock() const
    {
        return this->clock;
    }

    const Frame_format& get_frame_format() const
    {
        return this->frame_format;
    }

    constexpr Id get_id() const
    {
        return Id::_2;
    }

private:
    Transmit_callback transmit_callback;
    Receive_callback receive_callback;
    Bus_status_callback bus_status_callback;

    uint32_t baud_rate;

    Clock clock;
    Frame_format frame_format;

private:
    friend void usart_interrupt_handler(USART* a_p_this);
};

constexpr USART::Bus_status_flag operator|(USART::Bus_status_flag a_f1, USART::Bus_status_flag a_f2)
{
    return static_cast<USART::Bus_status_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr USART::Bus_status_flag operator&(USART::Bus_status_flag a_f1, USART::Bus_status_flag a_f2)
{
    return static_cast<USART::Bus_status_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr USART::Bus_status_flag operator|=(USART::Bus_status_flag& a_f1, USART::Bus_status_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr USART::Flow_control_flag operator|(USART::Flow_control_flag a_f1, USART::Flow_control_flag a_f2)
{
    return static_cast<USART::Flow_control_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr USART::Flow_control_flag operator&(USART::Flow_control_flag a_f1, USART::Flow_control_flag a_f2)
{
    return static_cast<USART::Flow_control_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr USART::Flow_control_flag operator|=(USART::Flow_control_flag& a_f1, USART::Flow_control_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr USART::Mode_flag operator|(USART::Mode_flag a_f1, USART::Mode_flag a_f2)
{
    return static_cast<USART::Mode_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr USART::Mode_flag operator&(USART::Mode_flag a_f1, USART::Mode_flag a_f2)
{
    return static_cast<USART::Mode_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr USART::Mode_flag operator|=(USART::Mode_flag& a_f1, USART::Mode_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc
