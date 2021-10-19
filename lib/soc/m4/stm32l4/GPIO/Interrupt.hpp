#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// soc
#include <soc/Handle.hpp>
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>
#include <soc/m4/stm32l4/IRQ.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<GPIO> : private cml::Non_copyable
{
public:
    enum class Id : std::uint32_t
    {
        _0     = EXTI0_IRQn,
        _1     = EXTI1_IRQn,
        _2     = EXTI2_IRQn,
        _3     = EXTI3_IRQn,
        _4     = EXTI4_IRQn,
        _5_9   = EXTI9_5_IRQn,
        _10_15 = EXTI15_10_IRQn
    };

    enum class Mode : std::uint32_t
    {
        interrupt,
        event
    };

    enum class Trigger_flag : std::uint32_t
    {
        rising  = 0x1,
        falling = 0x2,
    };

    struct id : private cml::Non_constructible
    {
        constexpr static auto _0     = Handle<IRQn_Type::EXTI0_IRQn> {};
        constexpr static auto _1     = Handle<IRQn_Type::EXTI1_IRQn> {};
        constexpr static auto _2     = Handle<IRQn_Type::EXTI2_IRQn> {};
        constexpr static auto _3     = Handle<IRQn_Type::EXTI3_IRQn> {};
        constexpr static auto _4     = Handle<IRQn_Type::EXTI4_IRQn> {};
        constexpr static auto _5_9   = Handle<IRQn_Type::EXTI9_5_IRQn> {};
        constexpr static auto _10_15 = Handle<IRQn_Type::EXTI15_10_IRQn> {};
    };

    struct Callback
    {
        using Function = void (*)(std::uint32_t a_pin, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Interrupt(Handle<IRQn_Type::EXTI0_IRQn>)
        : idx(0u)
        , irqn(IRQn_Type::EXTI0_IRQn)
    {
    }

    Interrupt(Handle<IRQn_Type::EXTI1_IRQn>)
        : idx(1u)
        , irqn(IRQn_Type::EXTI1_IRQn)
    {
    }

    Interrupt(Handle<IRQn_Type::EXTI2_IRQn>)
        : idx(2u)
        , irqn(IRQn_Type::EXTI2_IRQn)
    {
    }

    Interrupt(Handle<IRQn_Type::EXTI3_IRQn>)
        : idx(3u)
        , irqn(IRQn_Type::EXTI3_IRQn)
    {
    }

    Interrupt(Handle<IRQn_Type::EXTI4_IRQn>)
        : idx(4u)
        , irqn(IRQn_Type::EXTI4_IRQn)
    {
    }

    Interrupt(Handle<IRQn_Type::EXTI9_5_IRQn>)
        : idx(5u)
        , irqn(IRQn_Type::EXTI9_5_IRQn)
    {
    }

    Interrupt(Handle<IRQn_Type::EXTI15_10_IRQn>)
        : idx(6u)
        , irqn(IRQn_Type::EXTI15_10_IRQn)
    {
    }

    ~Interrupt()
    {
        this->disable();
    }

    void enable(const Callback& a_callback, const IRQ& a_irq);
    void disable();

    void attach(const GPIO& a_port, std::uint32_t a_pin, Trigger_flag a_trigger, Mode a_mode);
    void attach(const GPIO::In::Pin& a_pin, Trigger_flag a_trigger, Mode a_mode)
    {
        this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_mode);
    }
    void attach(const GPIO::Out::Pin& a_pin, Trigger_flag a_trigger, Mode a_mode)
    {
        this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_mode);
    }
    void attach(const GPIO::Alternate_function::Pin& a_pin, Trigger_flag a_trigger, Mode a_mode)
    {
        this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_mode);
    }

    void deattach(const GPIO& a_port, std::uint32_t a_pin);
    void deattach(const GPIO::In::Pin& a_pin)
    {
        this->deattach(*(a_pin.get_port()), a_pin.get_id());
    }
    void deattach(const GPIO::Out::Pin& a_pin)
    {
        this->deattach(*(a_pin.get_port()), a_pin.get_id());
    }
    void deattach(const GPIO::Alternate_function::Pin& a_pin)
    {
        this->deattach(*(a_pin.get_port()), a_pin.get_id());
    }

private:
    std::uint32_t idx;
    IRQn_Type irqn;
};

constexpr Interrupt<GPIO>::Trigger_flag operator|(Interrupt<GPIO>::Trigger_flag a_f1,
                                                  Interrupt<GPIO>::Trigger_flag a_f2)
{
    return static_cast<Interrupt<GPIO>::Trigger_flag>(static_cast<std::uint32_t>(a_f1) |
                                                      static_cast<std::uint32_t>(a_f2));
}
constexpr Interrupt<GPIO>::Trigger_flag operator&(Interrupt<GPIO>::Trigger_flag a_f1,
                                                  Interrupt<GPIO>::Trigger_flag a_f2)
{
    return static_cast<Interrupt<GPIO>::Trigger_flag>(static_cast<std::uint32_t>(a_f1) &
                                                      static_cast<std::uint32_t>(a_f2));
}
constexpr Interrupt<GPIO>::Trigger_flag operator|=(Interrupt<GPIO>::Trigger_flag& a_f1,
                                                   Interrupt<GPIO>::Trigger_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc