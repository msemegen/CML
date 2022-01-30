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

namespace soc {
void systick_interrupt_handler();

class Systick : private cml::Non_copyable
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = SysTick_CTRL_CLKSOURCE_Msk,
        _8 = 0
    };

    class Polling : private cml::Non_copyable
    {
    public:
        Polling(Polling&&) = default;
        Polling& operator=(Polling&&) = default;

        std::uint32_t get_value() const
        {
            return static_cast<SysTick_Type*>(*(this->p_systick))->VAL;
        }

    private:
        Polling()
            : p_systick(nullptr)
        {
        }

        Polling(Systick* a_p_systick)
            : p_systick(a_p_systick)
        {
        }

        Systick* p_systick;

        friend Systick;
    };
    class Interrupt : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        Interrupt(Interrupt&&) = default;
        Interrupt& operator=(Interrupt&&) = default;

#ifdef M4
        void enable(const m4::IRQ_config& a_irq_config);
#endif
        void disable();

        void register_callback(const Callback& a_callback);
        void unregister_callback();

    private:
        Interrupt()
            : p_systick(nullptr)
        {
        }

        Interrupt(Systick* a_p_systick)
            : p_systick(a_p_systick)
        {
        }

        Systick* p_systick;
        Callback callback;

        void friend soc::systick_interrupt_handler();
        friend Systick;
    };

    Systick(Systick&&) = default;
    Systick& operator=(Systick&&) = default;

    Systick()
        : polling(nullptr)
        , interrupt(nullptr)
        , idx(std::numeric_limits<decltype(this->idx)>::max())
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

    bool is_enabled();

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
        : polling(this)
        , interrupt(this)
        , idx(a_idx)
    {
    }

    std::uint32_t idx;

    template<typename Periph_t, std::size_t periph_id> friend class soc::Peripheral;
};

template<std::size_t id> class Peripheral<Systick, id> : private cml::Non_constructible
{
public:
    static Systick create()
    {
        return Systick(0);
    }
};
} // namespace soc