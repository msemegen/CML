#pragma once

/*
 *   Name: Systick.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>
#include <utility>

// externals
#ifdef STM32L4
#include <stm32l4xx.h>
#endif

// soc
#include <soc/Peripheral.hpp>
#ifdef M4
#include <soc/m4/IRQ_config.hpp>
#endif

// cml
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
class Systick : private cml::Non_copyable
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = SysTick_CTRL_CLKSOURCE_Msk,
        _8 = 0
    };

    class Polling
    {
    public:
        std::uint32_t get_value() const
        {
            return static_cast<SysTick_Type*>(*(this->p_systick))->VAL;
        }

    private:
        Systick* p_systick = nullptr;
        friend Systick;
    };
    class Interrupt
    {
    public:
        struct Callback
        {
            using Function = void (*)(void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

#ifdef M4
        void enable(const m4::IRQ_config& a_irq_config);
        void disable();
#endif
        void register_callback(const Callback& a_callback);
        void unregister_callback();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(IRQn_Type::SysTick_IRQn);
        }

    private:
        Systick* p_systick = nullptr;
        friend Systick;
    };

    Systick(Systick&&) = default;
    Systick& operator=(Systick&&) = default;

    Systick()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
    {
    }
    ~Systick()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable(std::uint32_t a_start_value, Prescaler a_prescaler);
    void disable();

    bool is_enabled()
    {
        return cml::bit_flag::is(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
    }

    std::uint32_t get_idx()
    {
        return this->idx;
    }

    operator SysTick_Type*()
    {
        return SysTick;
    }

    operator const SysTick_Type*() const
    {
        return SysTick;
    }

    Polling polling;
    Interrupt interrupt;

private:
    Systick(std::uint32_t a_idx)
        : idx(a_idx)
    {
        this->polling.p_systick   = this;
        this->interrupt.p_systick = this;
    }

    std::uint32_t idx;
    Interrupt::Callback callback;

    template<typename Periph_t, std::size_t periph_id> friend class soc::Peripheral;
    friend void systick_interrupt_handler();
};
void systick_interrupt_handler();

template<std::size_t id> class Peripheral<Systick, id> : private cml::Non_constructible
{
public:
    static Systick create()
    {
        return Systick(0);
    }
};
} // namespace soc