#pragma once

/*
    Name: RS485.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>
#include <type_traits>

// soc
#include <soc/stm32l452xx/peripherals/GPIO.hpp>
#include <soc/stm32l452xx/peripherals/USART.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l452xx {
namespace peripherals {

class RS485 : cml::Non_copyable
{
public:
    using Oversampling    = USART::Oversampling;
    using Stop_bits       = USART::Stop_bits;
    using Bus_status_flag = USART::Bus_status_flag;

    using Result = USART::Result;

    struct Transmit_callback
    {
        using Function = void (*)(volatile uint16_t* a_p_data,
                                  bool a_transfer_complete,
                                  RS485* a_p_this,
                                  void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Receive_callback
    {
        using Function = void (*)(uint32_t a_data, bool a_idle, RS485* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_status_callback
    {
        using Function = void (*)(Bus_status_flag a_bus_status, RS485* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Config
    {
        uint32_t baud_rate        = 0;
        Oversampling oversampling = Oversampling::unknown;
        Stop_bits stop_bits       = Stop_bits::unknown;
        uint8_t address           = 0;
    };

public:
    RS485(USART::Id a_id)
        : id(a_id)
        , p_flow_control_pin(nullptr)
    {
    }

    ~RS485()
    {
        this->disable();
    }

    bool enable(const Config& a_config,
                const USART::Clock& a_clock,
                GPIO::Out::Pin* a_p_flow_control_pin,
                uint32_t a_irq_priority,
                cml::time::tick a_timeout);

    void disable();

    template<typename Data_t> Result transmit_polling(uint8_t a_address, const Data_t& a_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(a_address, &a_data, sizeof(a_data));
    }

    template<typename Data_t>
    Result transmit_polling(uint8_t a_address, const Data_t& a_data, cml::time::tick a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(a_address, &a_data, sizeof(a_data), a_timeout);
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t));
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data, cml::time::tick a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t), a_timeout);
    }

    Result transmit_bytes_polling(uint8_t a_address, const void* a_p_data, uint32_t a_data_size_in_words);
    Result transmit_bytes_polling(uint8_t a_address,
                                  const void* a_p_data,
                                  uint32_t a_data_size_in_words,
                                  cml::time::tick a_timeout_ms);

    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words);
    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, cml::time::tick a_timeout_ms);

    void register_transmit_callback(const Transmit_callback& a_callback);
    void register_receive_callback(const Receive_callback& a_callback);
    void register_bus_status_callback(const Bus_status_callback& a_callback);

    void unregister_transmit_callback();
    void unregister_receive_callback();
    void unregister_bus_status_callback();

    void set_baud_rate(uint32_t a_baud_rate);
    void set_oversampling(Oversampling a_oversampling);
    void set_stop_bits(Stop_bits a_stop_bits);

    bool is_enabled() const;

    Oversampling get_oversampling() const;
    Stop_bits get_stop_bits() const;

    uint32_t get_baud_rate() const
    {
        return this->baud_rate;
    }

    const USART::Clock& get_clock() const
    {
        return this->clock;
    }

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

private:
    USART::Id id;
    GPIO::Out::Pin* p_flow_control_pin;

    Transmit_callback transmit_callback;
    Receive_callback receive_callback;
    Bus_status_callback bus_status_callback;

    uint32_t baud_rate;
    USART::Clock clock;

private:
    friend void rs485_interrupt_handler(RS485* a_p_this);
};

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc