#pragma once

/*
    Name: usart.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l4xx.h>

//cml
#include <common/assert.hpp>
#include <common/frequency.hpp>
#include <common/integer.hpp>
#include <common/time_tick.hpp>
#include <hal/systick.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class c_usart
{
public:

    enum class e_periph : common::uint32
    {
        _1 = 0,
        _2 = 1,
        _3 = 2
    };

    enum class e_baud_rate : common::uint32
    {
        _110    = 110,
        _300    = 300,
        _600    = 600,
        _1200   = 1200,
        _2400   = 2400,
        _4800   = 4800,
        _9600   = 9600,
        _14400  = 14400,
        _19200  = 19200,
        _38400  = 38400,
        _57600  = 57600,
        _115200 = 115200,
        _230400 = 230400,
        _460800 = 460800,
        _921600 = 921600,
        unknown
    };

    enum class e_oversampling : common::uint32
    {
        _8  = USART_CR1_OVER8,
        _16 = 0,
        unknown
    };

    enum class e_word_length : common::uint32
    {
        _7_bits = USART_CR1_M1,
        _8_bits = 0x0u,
        _9_bits = USART_CR1_M0,
        unknown
    };

    enum class e_stop_bits : common::uint32
    {
        _0_5 = USART_CR2_STOP_0,
        _1   = 0x0u,
        _1_5 = USART_CR2_STOP_0 | USART_CR2_STOP_1,
        _2   = USART_CR2_STOP_1,
        unknown
    };

    enum class e_flow_control : common::uint32
    {
        none                          = 0x0u,
        request_to_send               = USART_CR3_RTSE,
        clear_to_send                 = USART_CR3_CTSE,
        request_to_send_clear_to_send = USART_CR3_RTSE | USART_CR3_CTSE,
        unknown
    };

    enum class e_parity : common::uint32
    {
        none = 0x0u,
        even = USART_CR1_PCE,
        odd  = USART_CR1_PCE | USART_CR1_PS,
        unknown
    };


    enum class e_mode : common::uint32
    {
        rx      = USART_CR1_RE,
        tx      = USART_CR1_TE,
        rxtx    = USART_CR1_RE | USART_CR1_TE,
        unknown
    };

    struct s_config
    {
        e_baud_rate baud_rate       = e_baud_rate::unknown;
        e_oversampling oversampling = e_oversampling::unknown;
        e_word_length word_length   = e_word_length::unknown;
        e_stop_bits stop_bits       = e_stop_bits::unknown;
        e_flow_control flow_control = e_flow_control::unknown;
        e_parity parity             = e_parity::unknown;
        e_mode mode                 = e_mode::unknown;
    };

    struct s_clock
    {
        enum class e_source : common::uint32
        {
            pclk,
            sysclk,
            hsi,
            unknown
        };

        e_source source                = e_source::unknown;
        common::frequency frequency_hz = common::Hz(0);
    };

    struct s_tx_callback
    {
        using function = bool(*)(common::byte* p_a_byte, void* a_p_user_data, bool a_timeout);

        function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    struct s_rx_callback
    {
        using function = bool(*)(common::byte a_byte, void* a_p_user_data, bool a_timeout);

        function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

public:

    c_usart(e_periph a_periph)
        : periph(a_periph)
        , p_usart(nullptr)
        , baud_rate(e_baud_rate::unknown)
    {}

    ~c_usart()
    {
        this->disable();
    }

    c_usart()               = default;
    c_usart(c_usart&)       = default;
    c_usart(const c_usart&) = default;

    c_usart& operator = (c_usart&&)      = default;
    c_usart& operator = (const c_usart&) = default;

    void enable(const s_config& a_config, const s_clock& a_clock);
    void disable();

    template<typename data_t>
    void write_polling(const data_t& a_data)
    {
        this->write_bytes_polling(static_cast<const void*>(&a_data), sizeof(a_data));
    }

    template<typename data_t>
    bool write_polling(const data_t& a_data, common::time_tick a_timeout)
    {
        return this->write_bytes_polling(static_cast<const void*>(&a_data), sizeof(a_data), a_timeout);
    }

    template<typename data_t>
    void read_polling(data_t* a_p_data)
    {
        this->read_bytes_polling(static_cast<void*>(a_p_data), sizeof(data_t));
    }

    template<typename data_t>
    bool read_polling(data_t* a_p_data, common::time_tick a_timeout)
    {
        return this->write_bytes_polling(static_cast<void*>(&a_p_data), sizeof(data_t), a_timeout);
    }

    void write_bytes_polling(const void* a_p_data, common::uint32 a_data_length);
    bool write_bytes_polling(const void* a_p_data, common::uint32 a_data_length, common::time_tick a_timeout);

    void read_bytes_polling(void* a_p_data, common::uint32 a_data_length);
    bool read_bytes_polling(void* a_p_data, common::uint32 a_data_length, common::time_tick a_timeout);

    void write_bytes_it(const s_tx_callback& a_callback)
    {
        this->write_bytes_it(a_callback, common::time_tick_infinity);
    }
    void write_bytes_it(const s_tx_callback& a_callback, common::time_tick a_timeout);

    void read_bytes_it(const s_rx_callback& a_callback)
    {
        this->read_bytes_it(a_callback, common::time_tick_infinity);
    }
    void read_bytes_it(const s_rx_callback& a_callback, common::time_tick a_timeout);

    void set_baud_rate(e_baud_rate a_baud_rate);
    void set_oversampling(e_oversampling a_oversampling);
    void set_word_length(e_word_length a_word_length);
    void set_parity(e_parity a_parity);
    void set_stop_bits(e_stop_bits a_stop_bits);
    void set_flow_control(e_flow_control a_flow_control);
    void set_mode(e_mode a_mode);

    e_baud_rate    get_baud_rate()    const;
    e_oversampling get_oversampling() const;
    e_word_length  get_word_length()  const;
    e_stop_bits    get_stop_bits()    const;
    e_flow_control get_flow_control() const;
    e_mode         get_mode()         const;

    e_periph get_periph() const
    {
        return this->periph;
    }

private:

    template<typename callback_type>
    struct s_it_context
    {
        callback_type callback;
        common::time_tick timeout         = 0;
        common::time_tick start_timestamp = 0;
    };

private:

    common::uint32 to_index(e_periph a_periph) const
    {
        return static_cast<common::uint32>(a_periph);
    }

private:

    using s_tx_it_context = s_it_context<s_tx_callback>;
    using s_rx_it_context = s_it_context<s_rx_callback>;

private:

    e_periph periph;

    s_tx_it_context tx_context;
    s_rx_it_context rx_context;

    USART_TypeDef* p_usart;

    e_baud_rate baud_rate;

private:

    friend class c_interrupt_handler;
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml
