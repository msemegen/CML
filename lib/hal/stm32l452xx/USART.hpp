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
#include <common/Non_copyable.hpp>
#include <common/time.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class USART : private common::Non_copyable
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

    enum class Bus_status_flag : common::uint32
    {
        ok             = 0x0,
        framing_error  = 0x1,
        parity_error   = 0x2,
        overrun        = 0x4,
        noise_detected = 0x8
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
        using Function = bool(*)(volatile common::uint16* p_a_data, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct RX_callback
    {
        using Function = bool(*)(common::uint32 a_data, bool a_idle, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_status_callback
    {
        using Function = bool(*)(Bus_status_flag a_bus_status, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
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

    bool enable(const Config& a_config,
                const Clock& a_clock,
                common::uint32 a_irq_priority,
                common::time::tick a_timeout);

    void disable();

    template<typename Data_t>
    common::uint32 transmit_polling(const Data_t& a_data, Bus_status_flag* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(static_cast<const void*>(&a_data), sizeof(a_data), a_p_status);
    }

    template<typename Data_t>
    common::uint32 transmit_polling(const Data_t& a_data,
                                    common::time::tick a_timeout,
                                    Bus_status_flag* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(static_cast<const void*>(&a_data), sizeof(a_data), a_timeout, a_p_status);
    }

    template<typename Data_t>
    common::uint32 receive_polling(Data_t* a_p_data, Bus_status_flag* a_p_status = nullptr)
    {
        return this->receive_bytes_polling(static_cast<void*>(a_p_data), sizeof(Data_t), a_p_status);
    }

    template<typename Data_t>
    common::uint32 receive_polling(Data_t* a_p_data,
                                   common::time::tick a_timeout,
                                   Bus_status_flag* a_p_status = nullptr)
    {
        return this->receive_bytes_polling(static_cast<void*>(&a_p_data), sizeof(Data_t), a_timeout, a_p_status);
    }

    common::uint32 transmit_bytes_polling(const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          Bus_status_flag* a_p_status = nullptr);

    common::uint32 transmit_bytes_polling(const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          common::time::tick a_timeout,
                                          Bus_status_flag* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         Bus_status_flag* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         common::time::tick a_timeout,
                                         Bus_status_flag* a_p_status = nullptr);

    void register_transmit_callback(const TX_callback& a_callback);
    void register_receive_callback(const RX_callback& a_callback);
    void register_bus_status_callback(const Bus_status_callback& a_callback);

    void unregister_transmit_callback();
    void unregister_receive_callback();
    void unregister_bus_status_callback();

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

    Id get_id() const
    {
        return this->id;
    }

private:

    Id id;
    USART_TypeDef* p_usart;

    TX_callback tx_callback;
    RX_callback rx_callback;
    Bus_status_callback bus_status_callback;

    common::uint32 baud_rate;
    Clock clock;

private:

    friend void usart_interrupt_handler(USART* a_p_this);
};

constexpr USART::Bus_status_flag operator | (USART::Bus_status_flag a_f1, USART::Bus_status_flag a_f2)
{
    return static_cast<USART::Bus_status_flag>(static_cast<common::uint32>(a_f1) | static_cast<common::uint32>(a_f2));
}

constexpr USART::Bus_status_flag operator & (USART::Bus_status_flag a_f1, USART::Bus_status_flag a_f2)
{
    return static_cast<USART::Bus_status_flag>(static_cast<common::uint32>(a_f1) & static_cast<common::uint32>(a_f2));
}

constexpr USART::Bus_status_flag operator |= (USART::Bus_status_flag& a_f1, USART::Bus_status_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml
