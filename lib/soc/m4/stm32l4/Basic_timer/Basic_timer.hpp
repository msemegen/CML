#pragma once

/*
 *   Name: Basic_timer.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Peripheral.hpp>
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class Basic_timer : private cml::Non_copyable
{
public:
    struct Enable_config
    {
        enum class Mode : std::uint32_t
        {
            disabled,
            enabled
        };

        enum class Autoreload_preload : std::uint32_t
        {
            disabled = 0x0u,
            enabled  = TIM_CR1_ARPE,
        };

        Mode mode                              = cml::various::get_enum_incorrect_value<Mode>();
        Autoreload_preload auto_reload_preload = cml::various::get_enum_incorrect_value<Autoreload_preload>();

        std::uint16_t prescaler   = 0u;
        std::uint16_t auto_reload = 0u;
    };

    class Polling : private cml::Non_copyable
    {
    public:
        bool is_overload() const
        {
            return cml::bit::is(static_cast<TIM_TypeDef*>(*(this->p_timer))->CNT, 31u);
        }

        bool is_created() const
        {
            return nullptr != this->p_timer;
        }

    private:
        Basic_timer* p_timer = nullptr;
        friend Basic_timer;
    };
    class Interrupt : private cml::Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(Basic_timer* p_this, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void enable(const IRQ_config& a_irq_config);
        void disable();

        void register_callback(const Callback& a_callback);
        void unregister_callback();

        bool is_enabled() const
        {
            return 0u != NVIC_GetEnableIRQ(this->p_timer->irqn);
        }

    private:
        void set_irq_context();
        void clear_irq_context();

        Basic_timer* p_timer = nullptr;
        friend Basic_timer;
    };

    Basic_timer(Basic_timer&&) = default;
    Basic_timer& operator=(Basic_timer&&) = default;

    Basic_timer()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
    }
    ~Basic_timer()
    {
        if (true == this->is_enabled())
        {
            this->stop();
            this->disable();
        }
    }

    void enable(const Enable_config& a_config);
    void disable();

    void start();
    void stop();

    bool is_enabled() const
    {
        return 0x0u != this->p_registers->CR1;
    }

    bool is_ceated() const
    {
        return std::numeric_limits<decltype(this->idx)>::max() != this->idx && nullptr != this->p_registers;
    }

    operator TIM_TypeDef*()
    {
        return this->p_registers;
    }

    operator const TIM_TypeDef*() const
    {
        return this->p_registers;
    }

    Polling polling;
    Interrupt interrupt;

private:
    Basic_timer(std::size_t a_idx, TIM_TypeDef* a_p_registers, IRQn_Type a_irqn_type)
        : idx(0u)
        , p_registers(a_p_registers)
    {
        this->polling.p_timer   = this;
        this->interrupt.p_timer = this;
    }

    const std::uint32_t idx;
    TIM_TypeDef* p_registers;

    IRQn_Type irqn;
    Interrupt::Callback callback;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
    friend void Basic_timer_interrupt_handler(Basic_timer* a_p_this);
};
void Basic_timer_interrupt_handler(Basic_timer* a_p_this);

template<std::size_t id> class rcc<Basic_timer, id> : private cml::Non_constructible
{
public:
    static void enable(bool a_enable_in_lp) = delete;
    static void disable()                   = delete;
};

void Basic_timer_interrupt_handler(Basic_timer* a_p_this);
} // namespace stm32l4
} // namespace m4
} // namespace soc
