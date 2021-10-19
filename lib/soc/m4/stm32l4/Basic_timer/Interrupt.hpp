#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/Basic_timer/Basic_timer.hpp>
#include <soc/m4/stm32l4/IRQ.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>

// soc
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<Basic_timer> : private cml::Non_copyable
{
public:
    struct Overload_callback
    {
        using Function = void (*)(Basic_timer* p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Interrupt(Basic_timer* a_p_timer, Handle<TIM6_BASE>)
        : p_timer(a_p_timer)
        , irqn(IRQn_Type::TIM6_DAC_IRQn)
    {
    }
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
    Interrupt(Basic_timer* a_p_timer, Handle<TIM7_BASE>)
        : p_timer(a_p_timer)
        , irqn(IRQn_Type::TIM7_IRQn)
    {
    }
#endif
    ~Interrupt();

    void enable(const IRQ& a_irq);
    void disable();

    void register_callback(const Overload_callback& a_callback);

    Basic_timer* get_handle()
    {
        return this->p_timer;
    }

    const Basic_timer* get_handle() const
    {
        return this->p_timer;
    }

private:
    Basic_timer* p_timer;
    IRQn_Type irqn;

    Overload_callback callback;

private:
    friend void basic_timer_interrupt_handler(Interrupt<Basic_timer>* a_p_this);
};
} // namespace stm32l4
} // namespace m4
} // namespace soc