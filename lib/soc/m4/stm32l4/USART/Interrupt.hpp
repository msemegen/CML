#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/IRQ.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>
#include <soc/m4/stm32l4/USART/USART.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<USART> : private cml::Non_copyable
{
public:
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
        using Function = void (*)(std::uint32_t a_data, bool a_idle, USART* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_status_callback
    {
        enum class Status_flag : std::uint32_t
        {
            ok             = 0x0,
            framing_error  = 0x1,
            parity_error   = 0x2,
            overrun        = 0x4,
            noise_detected = 0x8,
        };

        using Function = void (*)(Status_flag a_bus_status, USART* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Interrupt(USART* a_p_usart, Handle<USART1_BASE>)
        : p_usart(a_p_usart)
        , irqn(IRQn_Type::USART1_IRQn)
    {
    }
    Interrupt(USART* a_p_usart, Handle<USART2_BASE>)
        : p_usart(a_p_usart)
        , irqn(IRQn_Type::USART2_IRQn)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    Interrupt(USART* a_p_usart, Handle<USART3_BASE>)
        : p_usart(a_p_usart)
        , irqn(IRQn_Type::USART3_IRQn)
    {
    }
#endif

    void enable(const IRQ& a_irq);
    void disable();

    void register_callback(const Transmit_callback& a_callback);
    void register_callback(const Receive_callback& a_callback);
    void register_callback(const Bus_status_callback& a_callback);

    USART* get_handle()
    {
        return this->p_usart;
    }

    const USART* get_handle() const
    {
        return this->p_usart;
    }

private:
    USART* p_usart;
    IRQn_Type irqn;

    Transmit_callback transmit_callback;
    Receive_callback receive_callback;
    Bus_status_callback bus_status_callback;

private:
    friend void usart_interrupt_handler(Interrupt<USART>* a_p_this);
};

constexpr Interrupt<USART>::Bus_status_callback::Status_flag
operator|(Interrupt<USART>::Bus_status_callback::Status_flag a_f1,
          Interrupt<USART>::Bus_status_callback::Status_flag a_f2)
{
    return static_cast<Interrupt<USART>::Bus_status_callback::Status_flag>(static_cast<std::uint32_t>(a_f1) |
                                                                           static_cast<std::uint32_t>(a_f2));
}

constexpr Interrupt<USART>::Bus_status_callback::Status_flag
operator&(Interrupt<USART>::Bus_status_callback::Status_flag a_f1,
          Interrupt<USART>::Bus_status_callback::Status_flag a_f2)
{
    return static_cast<Interrupt<USART>::Bus_status_callback::Status_flag>(static_cast<std::uint32_t>(a_f1) &
                                                                           static_cast<std::uint32_t>(a_f2));
}

constexpr Interrupt<USART>::Bus_status_callback::Status_flag
operator|=(Interrupt<USART>::Bus_status_callback::Status_flag& a_f1,
           Interrupt<USART>::Bus_status_callback::Status_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc