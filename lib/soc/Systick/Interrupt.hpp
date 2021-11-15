#pragma once

/*
 *   Name: Interrupt.Hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/Factory.hpp>
#include <soc/Systick/Systick.hpp>
#ifdef M4
#include <soc/m4/IRQ_config.hpp>
#endif
#ifdef STM32L4
#include <soc/m4/stm32l4/Interrupt.hpp>
#endif

namespace soc {
void systick_interrupt_handler();

template<> class m4::stm32l4::Interrupt<Systick> : private cml::Non_copyable
{
public:
    struct Callback
    {
        using Function = void (*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    Interrupt()
        : p_systick(nullptr)
    {
    }

#ifdef M4
    void enable(const m4::IRQ_config& a_irq_config);
#endif
    void disable();

    void register_callback(const Callback& a_callback);
    void unregister_callback();

    Systick* get_handle()
    {
        return this->p_systick;
    }

    const Systick* get_handle() const
    {
        return this->p_systick;
    }

private:
    Interrupt(Systick* a_p_systick)
        : p_systick(a_p_systick)
    {
    }

    Systick* p_systick;
    Callback callback;

    void friend soc::systick_interrupt_handler();
    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

template<> class Factory<m4::stm32l4::Interrupt<Systick>> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Interrupt<Systick> create(Systick* a_p_systick)
    {
        return m4::stm32l4::Interrupt<Systick>(a_p_systick);
    }
};
} // namespace soc