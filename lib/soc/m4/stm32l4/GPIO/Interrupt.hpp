#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>

// soc
#include <soc/Factory.hpp>
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>
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
        _0,
        _1,
        _2,
        _3,
        _4,
        _5_9,
        _10_15
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

    struct Callback
    {
        using Function = void (*)(std::uint32_t a_pin, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    Interrupt()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<int32_t>::max()))
    {
    }

    ~Interrupt()
    {
        if (0x0 != NVIC_GetEnableIRQ(this->irqn))
        {
            this->disable();
        }
    }

    void enable(const Callback& a_callback, const IRQ_config& a_irq_config);
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
    Interrupt(std::size_t a_idx, IRQn_Type a_irqn)
        : idx(a_idx)
        , irqn(a_irqn)
    {
    }

    std::uint32_t idx;
    IRQn_Type irqn;

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
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