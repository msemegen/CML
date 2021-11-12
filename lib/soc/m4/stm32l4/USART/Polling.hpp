#pragma once

/*
 *   Name: Polling.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstddef>

// soc
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>
#include <soc/m4/stm32l4/Polling.hpp>
#include <soc/m4/stm32l4/USART/RS485.hpp>
#include <soc/m4/stm32l4/USART/USART.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Polling<USART> : private cml::Non_copyable
{
public:
    struct Result
    {
        enum class Bus_status_flag : std::uint32_t
        {
            ok             = 0x0,
            framing_error  = 0x1,
            parity_error   = 0x2,
            overrun        = 0x4,
            noise_detected = 0x8,
        };

        Bus_status_flag bus_status       = cml::various::get_enum_incorrect_value<Bus_status_flag>();
        std::size_t data_length_in_words = 0;
    };

public:
    Result transmit(const void* a_p_data, std::size_t a_data_size_in_words);
    Result transmit(const void* a_p_data, std::size_t a_data_size_in_words, std::uint32_t a_timeout_ms);

    Result receive(void* a_p_data, std::size_t a_data_size_in_words);
    Result receive(void* a_p_data, std::size_t a_data_size_in_words, std::uint32_t a_timeout_ms);

    USART* get_USART()
    {
        return this->p_usart;
    }

    const USART* get_handle() const
    {
        return this->p_usart;
    }

private:
    Polling(USART* a_p_usart)
        : p_usart(a_p_usart)
    {
    }

private:
    USART* p_usart;

private:
    template<typename Periph_t, std::size_t id> friend class Factory;
};

template<> class Polling<RS485> : private cml::Non_copyable
{
public:
    struct Result
    {
        enum class Bus_status_flag : std::uint32_t
        {
            ok             = 0x0,
            framing_error  = 0x1,
            parity_error   = 0x2,
            overrun        = 0x4,
            noise_detected = 0x8,
        };

        Bus_status_flag bus_status       = cml::various::get_enum_incorrect_value<Bus_status_flag>();
        std::size_t data_length_in_words = 0;
    };

public:
    Result transmit(std::uint8_t a_address, const void* a_p_data, std::size_t a_data_size_in_words);
    Result transmit(std::uint8_t a_address,
                    const void* a_p_data,
                    std::size_t a_data_size_in_words,
                    std::uint32_t a_timeout_ms);

    Result receive(void* a_p_data, std::size_t a_data_size_in_words);
    Result receive(void* a_p_data, std::size_t a_data_size_in_words, std::uint32_t a_timeout_ms);

private:
    Polling(RS485* a_p_RS485, GPIO::Out::Pin* a_p_flow_control_pin)
        : p_RS485(a_p_RS485)
        , p_flow_control_pin(a_p_flow_control_pin)
    {
    }

private:
    RS485* p_RS485;
    GPIO::Out::Pin* p_flow_control_pin;

private:
    template<typename Periph_t, std::size_t id> friend class Factory;
};

constexpr Polling<USART>::Result::Bus_status_flag operator|(Polling<USART>::Result::Bus_status_flag a_f1,
                                                            Polling<USART>::Result::Bus_status_flag a_f2)
{
    return static_cast<Polling<USART>::Result::Bus_status_flag>(static_cast<std::uint32_t>(a_f1) |
                                                                static_cast<std::uint32_t>(a_f2));
}

constexpr Polling<USART>::Result::Bus_status_flag operator&(Polling<USART>::Result::Bus_status_flag a_f1,
                                                            Polling<USART>::Result::Bus_status_flag a_f2)
{
    return static_cast<Polling<USART>::Result::Bus_status_flag>(static_cast<std::uint32_t>(a_f1) &
                                                                static_cast<std::uint32_t>(a_f2));
}

constexpr Polling<USART>::Result::Bus_status_flag operator|=(Polling<USART>::Result::Bus_status_flag& a_f1,
                                                             Polling<USART>::Result::Bus_status_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr Polling<RS485>::Result::Bus_status_flag operator|(Polling<RS485>::Result::Bus_status_flag a_f1,
                                                            Polling<RS485>::Result::Bus_status_flag a_f2)
{
    return static_cast<Polling<RS485>::Result::Bus_status_flag>(static_cast<std::uint32_t>(a_f1) |
                                                                static_cast<std::uint32_t>(a_f2));
}

constexpr Polling<RS485>::Result::Bus_status_flag operator&(Polling<RS485>::Result::Bus_status_flag a_f1,
                                                            Polling<RS485>::Result::Bus_status_flag a_f2)
{
    return static_cast<Polling<RS485>::Result::Bus_status_flag>(static_cast<std::uint32_t>(a_f1) &
                                                                static_cast<std::uint32_t>(a_f2));
}

constexpr Polling<RS485>::Result::Bus_status_flag operator|=(Polling<RS485>::Result::Bus_status_flag& a_f1,
                                                             Polling<RS485>::Result::Bus_status_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc