/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/GPIO/Interrupt.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/mcu/mcu.hpp>

// cml
#include <cml/bit.hpp>

namespace {

using namespace soc::m4::stm32l4;

Interrupt<GPIO>::Callback callbacks[7u];

} // namespace

extern "C" {

using namespace cml;

static bool interrupt_handler(std::uint32_t a_pr1, std::uint32_t a_bit_index, std::uint32_t a_handler_index)
{
    if (true == bit::is(a_pr1, a_bit_index))
    {
        callbacks[a_handler_index].function(a_bit_index, callbacks[a_handler_index].p_user_data);
        return true;
    }

    return false;
}

void EXTI0_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 0u, 0u))
    {
        bit::set(&(EXTI->PR1), 0u);
    }
}

void EXTI1_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 1u, 1u))
    {
        bit::set(&(EXTI->PR1), 1u);
    }
}

void EXTI2_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 2u, 2u))
    {
        bit::set(&(EXTI->PR1), 2u);
    }
}

void EXTI3_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 3u, 3u))
    {
        bit::set(&(EXTI->PR1), 3u);
    }
}

void EXTI4_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 4u, 4u))
    {
        bit::set(&(EXTI->PR1), 4u);
    }
}

void EXTI9_5_IRQHandler()
{
    for (std::uint32_t i = 5u; i <= 9u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i, 5u))
        {
            bit::set(&(EXTI->PR1), i);
        }
    }
}

void EXTI15_10_IRQHandler()
{
    for (std::uint32_t i = 10u; i <= 15u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i, 6u))
        {
            bit::set(&(EXTI->PR1), i);
        }
    }
}
}

namespace soc {
namespace m4 {
namespace stm32l4 {
void Interrupt<GPIO>::enable(const Callback& a_callback, const IRQ_config& a_irq_config)
{
    cml_assert(true == rcc<mcu>::is_SYSCFG_active());

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    callbacks[this->idx] = a_callback;
}

void Interrupt<GPIO>::disable()
{
    NVIC_DisableIRQ(this->irqn);

    callbacks[this->idx] = { nullptr, nullptr };
}

void Interrupt<GPIO>::attach(const GPIO& a_port, std::uint32_t a_pin, Trigger_flag a_trigger, Mode a_mode)
{
    volatile std::uint32_t* p_register = &(SYSCFG->EXTICR[a_pin / 4u]);
    std::uint32_t pos                  = ((static_cast<std::uint32_t>(a_pin) % 4u) * 4u);

#ifdef CML_ASSERT_ENABLED
    const bool f = bit_flag::is(*p_register, (a_port.get_id()) << pos);
    cml_assert((0u == a_port.get_id() && true == f) || (0u != a_port.get_id() && false == f));
    cml_assert((0u == this->idx && 0u == a_pin) || (1u == this->idx && 1u == a_pin) ||
               (2u == this->idx && 2u == a_pin) || (3u == this->idx && 3u == a_pin) ||
               (4u == this->idx && 4u == a_pin) || (5u == this->idx && (a_pin >= 5u && a_pin <= 9u)) ||
               (6u == this->idx && (a_pin >= 10u && a_pin <= 15u)));
#endif

    Interrupt_guard guard;

    bit_flag::set(p_register, 0x3u << pos, a_port.get_id() << pos);

    bit::clear(&(EXTI->RTSR1), a_pin);
    bit::clear(&(EXTI->FTSR1), a_pin);

    switch (a_mode)
    {
        case Mode::event: {
            bit::set(&(EXTI->EMR1), a_pin);
        }
        break;

        case Mode::interrupt: {
            bit::set(&(EXTI->IMR1), a_pin);
        }
        break;
    }

    switch (a_trigger)
    {
        case Trigger_flag::rising: {
            bit::set(&(EXTI->RTSR1), a_pin);
        }
        break;

        case Trigger_flag::falling: {
            bit::set(&(EXTI->FTSR1), a_pin);
        }
        break;

        default: {
            if ((Trigger_flag::rising | Trigger_flag::falling) == a_trigger)
            {
                bit::set(&(EXTI->RTSR1), a_pin);
                bit::set(&(EXTI->FTSR1), a_pin);
            }
        }
    }
}

void Interrupt<GPIO>::deattach(const GPIO& a_port, std::uint32_t a_pin)
{
    Interrupt_guard guard;

    bit::clear(&(EXTI->RTSR1), a_pin);
    bit::clear(&(EXTI->FTSR1), a_pin);

    bit::clear(&(EXTI->EMR1), a_pin);
    bit::clear(&(EXTI->IMR1), a_pin);

    bit_flag::clear(&(SYSCFG->EXTICR[a_pin / 4u]), a_port.get_id() << ((static_cast<std::uint32_t>(a_pin) % 4u) * 4u));

    callbacks[this->idx] = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif