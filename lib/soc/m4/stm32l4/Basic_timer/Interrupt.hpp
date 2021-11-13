#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/Basic_timer/Basic_timer.hpp>
#include <soc/m4/stm32l4/IRQ_config.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>

// soc
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<Basic_timer> : private cml::Non_copyable
{
public:
    struct Callback
    {
        using Function = void (*)(Basic_timer* p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    ~Interrupt()
    {
        this->disable();
    }

    void enable(const IRQ_config& a_irq_config);
    void disable();

    void register_callback(const Callback& a_callback);

    Basic_timer* get_handle()
    {
        return this->p_timer;
    }

    const Basic_timer* get_handle() const
    {
        return this->p_timer;
    }

private:
    Interrupt(Basic_timer* a_p_timer, IRQn_Type a_irqn)
        : p_timer(a_p_timer)
        , irqn(a_irqn)
    {
    }

    void set_irq_context();
    void clear_irq_context();

    Basic_timer* p_timer;
    IRQn_Type irqn;

    Callback callback;

    template<typename Periph_t, std::size_t id> friend class Factory;
    friend void basic_timer_interrupt_handler(Interrupt<Basic_timer>* a_p_this);
};
} // namespace stm32l4
} // namespace m4
} // namespace soc