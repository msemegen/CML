#pragma once

/*
    Name: USART.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l4xx.h>

//cml
#include <common/bit.hpp>
#include <common/frequency.hpp>
#include <common/integer.hpp>
#include <common/time_tick.hpp>
#include <debug/assert.hpp>
#include <hal/systick.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class USART
{
public:

    enum class Id : common::uint32
    {
        _1 = 0,
        _2 = 1,
        _3 = 2
    };

    enum class Oversampling : common::uint32
    {
        _8  = USART_CR1_OVER8,
        _16 = 0,
        unknown
    };

    enum class Word_length : common::uint32
    {
        _7_bits = USART_CR1_M1,
        _8_bits = 0x0u,
        _9_bits = USART_CR1_M0,
        unknown
    };

    enum class Stop_bits : common::uint32
    {
        _0_5 = USART_CR2_STOP_0,
        _1   = 0x0u,
        _1_5 = USART_CR2_STOP_0 | USART_CR2_STOP_1,
        _2   = USART_CR2_STOP_1,
        unknown
    };

    enum class Flow_control : common::uint32
    {
        none                          = 0x0u,
        request_to_send               = USART_CR3_RTSE,
        clear_to_send                 = USART_CR3_CTSE,
        request_to_send_clear_to_send = USART_CR3_RTSE | USART_CR3_CTSE,
        unknown
    };

    enum class Parity : common::uint32
    {
        none = 0x0u,
        even = USART_CR1_PCE,
        odd  = USART_CR1_PCE | USART_CR1_PS,
        unknown
    };

    struct Config
    {
        common::uint32 baud_rate  = 0;
        Oversampling oversampling = Oversampling::unknown;
        Word_length word_length   = Word_length::unknown;
        Stop_bits stop_bits       = Stop_bits::unknown;
        Flow_control flow_control = Flow_control::unknown;
        Parity parity             = Parity::unknown;
    };

    struct Clock
    {
        enum class Source : common::uint32
        {
            pclk,
            sysclk,
            hsi,
            unknown
        };

        Source source                  = Source::unknown;
        common::frequency frequency_hz = common::Hz(0);
    };

    struct TX_callback
    {
        using Function = bool(*)(common::byte* p_a_byte, void* a_p_user_data, bool a_timeout);

        Function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    struct RX_callback
    {
        using Function = bool(*)(common::byte a_byte, void* a_p_user_data, bool a_timeout);

        Function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

public:

    USART(Id a_id)
        : id(a_id)
        , p_usart(nullptr)
        , baud_rate(0)
    {}

    ~USART()
    {
        this->disable();
    }

    USART()             = default;
    USART(USART&)       = default;
    USART(const USART&) = default;

    USART& operator = (USART&&)      = default;
    USART& operator = (const USART&) = default;

    bool enable(const Config& a_config, const Clock& a_clock, common::time_tick a_timeout_ms);
    void disable();

    template<typename Data_t>
    void write_polling(const Data_t& a_data)
    {
        this->write_bytes_polling(static_cast<const void*>(&a_data), sizeof(a_data));
    }

    template<typename Data_t>
    bool write_polling(const Data_t& a_data, common::time_tick a_timeout_ms)
    {
        return this->write_bytes_polling(static_cast<const void*>(&a_data), sizeof(a_data), a_timeout_ms);
    }

    template<typename Data_t>
    void read_polling(Data_t* a_p_data)
    {
        this->read_bytes_polling(static_cast<void*>(a_p_data), sizeof(Data_t));
    }

    template<typename Data_t>
    bool read_polling(Data_t* a_p_data, common::time_tick a_timeout_ms)
    {
        return this->write_bytes_polling(static_cast<void*>(&a_p_data), sizeof(Data_t), a_timeout_ms);
    }

    void write_bytes_polling(const void* a_p_data, common::uint32 a_data_size_in_bytes);
    bool write_bytes_polling(const void* a_p_data, common::uint32 a_data_size_in_bytes, common::time_tick a_timeout_ms);

    void read_bytes_polling(void* a_p_data, common::uint32 a_data_size_in_bytes);
    bool read_bytes_polling(void* a_p_data, common::uint32 a_data_size_in_bytes, common::time_tick a_timeout_ms);

    void write_bytes_it(const TX_callback& a_callback)
    {
        this->write_bytes_it(a_callback, common::time_tick_infinity);
    }
    void write_bytes_it(const TX_callback& a_callback, common::time_tick a_timeout_ms);

    void read_bytes_it(const RX_callback& a_callback)
    {
        this->read_bytes_it(a_callback, common::time_tick_infinity);
    }
    void read_bytes_it(const RX_callback& a_callback, common::time_tick a_timeout_ms);

    void set_baud_rate(common::uint32 a_baud_rate);
    void set_oversampling(Oversampling a_oversampling);
    void set_word_length(Word_length a_word_length);
    void set_parity(Parity a_parity);
    void set_stop_bits(Stop_bits a_stop_bits);
    void set_flow_control(Flow_control a_flow_control);

    Oversampling get_oversampling() const;
    Word_length  get_word_length()  const;
    Stop_bits    get_stop_bits()    const;
    Flow_control get_flow_control() const;

    common::uint32 get_baud_rate() const
    {
        return this->baud_rate;
    }

    Clock get_clock() const
    {
        return this->clock;
    }

    bool is_rx_it_enabled() const;
    bool is_tx_it_enabled() const;

    Id get_id() const
    {
        return this->id;
    }

private:

    template<typename Callback_t>
    struct IT_context
    {
        Callback_t callback;
        common::time_tick timeout         = 0;
        common::time_tick start_timestamp = 0;
    };

private:

    common::uint32 to_index(Id a_id) const
    {
        return static_cast<common::uint32>(a_id);
    }

    bool is_isr_flag(common::uint32 a_flag)
    {
        return common::is_flag(this->p_usart->ISR, a_flag);
    }

    bool wait_until_isr(common::uint32 a_flag, bool a_status, common::time_tick a_start, common::time_tick a_timeout_ms)
    {
        bool status  = true;
        bool timeout = false;

        while (false == status && false == timeout)
        {
            timeout = a_timeout_ms >= common::time_tick_diff(systick::get_counter(), a_start);
            status  = this->is_isr_flag(a_flag) == a_status;
        }

        return ((true == status) && (false == timeout));
    }

    void wait_until_isr(common::uint32 a_flag, bool a_status)
    {
        while (this->is_isr_flag(a_flag) == a_status);
    }

private:

    using TX_it_context = IT_context<TX_callback>;
    using RX_it_context = IT_context<RX_callback>;

private:

    Id id;

    TX_it_context TX_context;
    RX_it_context RX_context;

    USART_TypeDef* p_usart;

    common::uint32 baud_rate;
    Clock clock;

private:

    friend void usart_handle_interrupt(USART* a_p_this);
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml
