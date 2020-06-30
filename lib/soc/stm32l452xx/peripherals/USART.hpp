#pragma once

/*
    Name: USART.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l4xx.h>

//cml
#include <cml/bit.hpp>
#include <cml/frequency.hpp>
#include <cml/integer.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/time.hpp>
#include <cml/type_traits.hpp>

namespace soc {
namespace stm32l452xx {
namespace peripherals {

class USART : private cml::Non_copyable
{
public:

    enum class Id : cml::uint32
    {
        _1 = 0,
        _2 = 1,
        _3 = 2
    };

    enum class Oversampling : cml::uint32
    {
        _8  = USART_CR1_OVER8,
        _16 = 0,
        unknown
    };

    enum class Stop_bits : cml::uint32
    {
        _0_5 = USART_CR2_STOP_0,
        _1   = 0x0u,
        _1_5 = USART_CR2_STOP_0 | USART_CR2_STOP_1,
        _2   = USART_CR2_STOP_1,
        unknown
    };

    enum class Flow_control_flag : cml::uint32
    {
        none            = 0x0u,
        request_to_send = USART_CR3_RTSE,
        clear_to_send   = USART_CR3_CTSE,
        unknown
    };

    enum class Parity : cml::uint32
    {
        none = 0x0u,
        even = USART_CR1_PCE,
        odd  = USART_CR1_PCE | USART_CR1_PS,
        unknown
    };

    enum Sampling_method : cml::uint32
    {
        three_sample_bit = 0,
        one_sample_bit   = USART_CR3_ONEBIT,
        unknown
    };

    enum class Bus_status_flag : cml::uint32
    {
        ok             = 0x0,
        framing_error  = 0x1,
        parity_error   = 0x2,
        overrun        = 0x4,
        noise_detected = 0x8,
        unknown        = 0x10
    };

    struct Config
    {
        cml::uint32 baud_rate           = 0;
        Oversampling oversampling       = Oversampling::unknown;
        Stop_bits stop_bits             = Stop_bits::unknown;
        Flow_control_flag flow_control  = Flow_control_flag::unknown;
        Parity parity                   = Parity::unknown;
        Sampling_method sampling_method = Sampling_method::unknown;
    };

    struct Clock
    {
        enum class Source : cml::uint32
        {
            pclk,
            sysclk,
            hsi,
            unknown
        };

        Source source               = Source::unknown;
        cml::frequency frequency_hz = cml::Hz(0);
    };

    struct Result
    {
        Bus_status_flag bus_status = Bus_status_flag::unknown;
        cml::uint32 data_length    = 0;
    };

    struct TX_callback
    {
        using Function = bool(*)(volatile cml::uint16* a_p_data, bool a_transfer_complete, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct RX_callback
    {
        using Function = bool(*)(cml::uint32 a_data, bool a_idle, void* a_p_user_data);

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
                cml::uint32 a_irq_priority,
                cml::time::tick a_timeout);

    void disable();

    template<typename Data_t>
    Result transmit_polling(const Data_t& a_data)
    {
        static_assert(true == cml::is_pod<Data_t>());
        return this->transmit_bytes_polling(&a_data, sizeof(a_data));
    }

    template<typename Data_t>
    Result transmit_polling(const Data_t& a_data, cml::time::tick a_timeout)
    {
        static_assert(true == cml::is_pod<Data_t>());
        return this->transmit_bytes_polling(&a_data, sizeof(a_data), a_timeout);
    }

    template<typename Data_t>
    Result receive_polling(Data_t* a_p_data)
    {
        static_assert(true == cml::is_pod<Data_t>());
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t));
    }

    template<typename Data_t>
    Result receive_polling(Data_t* a_p_data, cml::time::tick a_timeout)
    {
        static_assert(true == cml::is_pod<Data_t>());
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t), a_timeout);
    }

    Result transmit_bytes_polling(const void* a_p_data, cml::uint32 a_data_size_in_bytes);
    Result transmit_bytes_polling(const void* a_p_data, cml::uint32 a_data_size_in_bytes, cml::time::tick a_timeout);
    Result receive_bytes_polling(void* a_p_data, cml::uint32 a_data_size_in_bytes);
    Result receive_bytes_polling(void* a_p_data, cml::uint32 a_data_size_in_bytes, cml::time::tick a_timeout);

    void register_transmit_callback(const TX_callback& a_callback);
    void register_receive_callback(const RX_callback& a_callback);
    void register_bus_status_callback(const Bus_status_callback& a_callback);

    void unregister_transmit_callback();
    void unregister_receive_callback();
    void unregister_bus_status_callback();

    void set_baud_rate(cml::uint32 a_baud_rate);
    void set_oversampling(Oversampling a_oversampling);
    void set_parity(Parity a_parity);
    void set_stop_bits(Stop_bits a_stop_bits);
    void set_flow_control(Flow_control_flag a_flow_control);
    void set_sampling_method(Sampling_method a_sampling_method);

    bool is_transmit_callback_registered() const
    {
        return nullptr != this->tx_callback.function;
    }

    bool is_receive_callback_registered() const
    {
        return nullptr != this->rx_callback.function;
    }

    bool is_bus_status_callback_registered() const
    {
        return nullptr != this->tx_callback.function;
    }

    Oversampling      get_oversampling()    const;
    Stop_bits         get_stop_bits()       const;
    Flow_control_flag get_flow_control()    const;
    Sampling_method   get_sampling_method() const;

    bool is_enabled() const;

    cml::uint32 get_baud_rate() const
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

    cml::uint32 baud_rate;
    Clock clock;

private:

    friend void usart_interrupt_handler(USART* a_p_this);
};

constexpr USART::Bus_status_flag operator | (USART::Bus_status_flag a_f1, USART::Bus_status_flag a_f2)
{
    return static_cast<USART::Bus_status_flag>(static_cast<cml::uint32>(a_f1) | static_cast<cml::uint32>(a_f2));
}

constexpr USART::Bus_status_flag operator & (USART::Bus_status_flag a_f1, USART::Bus_status_flag a_f2)
{
    return static_cast<USART::Bus_status_flag>(static_cast<cml::uint32>(a_f1) & static_cast<cml::uint32>(a_f2));
}

constexpr USART::Bus_status_flag operator |= (USART::Bus_status_flag& a_f1, USART::Bus_status_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr USART::Flow_control_flag operator | (USART::Flow_control_flag a_f1, USART::Flow_control_flag a_f2)
{
    return static_cast<USART::Flow_control_flag>(static_cast<cml::uint32>(a_f1) | static_cast<cml::uint32>(a_f2));
}

constexpr USART::Flow_control_flag operator & (USART::Flow_control_flag a_f1, USART::Flow_control_flag a_f2)
{
    return static_cast<USART::Flow_control_flag>(static_cast<cml::uint32>(a_f1) & static_cast<cml::uint32>(a_f2));
}

constexpr USART::Flow_control_flag operator |= (USART::Flow_control_flag& a_f1, USART::Flow_control_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc
