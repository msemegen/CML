/*
 *   Name: EXTI.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/EXTI.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>

namespace {

using namespace soc::m4::stm32l4;
using namespace soc::m4::stm32l4::peripherals;

struct Interrupt_handler
{
    EXTI<GPIO>::Callback callback;
};

Interrupt_handler interrupt_handlers[7];

} // namespace

extern "C" {

using namespace cml;

#define EXTI_T ((EXTI_TypeDef*)EXTI_BASE)

static bool interrupt_handler(uint32_t a_pr1, uint32_t a_bit_index, uint32_t a_handler_index)
{
    if (true == bit::is(a_pr1, a_bit_index))
    {
        interrupt_handlers[a_handler_index].callback.function(a_bit_index,
                                                              interrupt_handlers[a_handler_index].callback.p_user_data);
        return true;
    }

    return false;
}

void EXTI0_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 0, 0))
    {
        bit::set(&(EXTI_T->PR1), 0);
    }
}

void EXTI1_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 1, 1))
    {
        bit::set(&(EXTI_T->PR1), 1);
    }
}

void EXTI2_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 2, 2))
    {
        bit::set(&(EXTI_T->PR1), 2);
    }
}

void EXTI3_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 3, 3))
    {
        bit::set(&(EXTI_T->PR1), 3);
    }
}

void EXTI4_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 4, 4))
    {
        bit::set(&(EXTI_T->PR1), 4);
    }
}

void EXTI9_5_IRQHandler()
{
    for (uint32_t i = 5u; i <= 9u; i++)
    {
        if (true == interrupt_handler(EXTI_T->PR1, i, 5))
        {
            bit::set(&(EXTI_T->PR1), i);
        }
    }
}

void EXTI15_10_IRQHandler()
{
    for (uint32_t i = 10u; i <= 15u; i++)
    {
        if (true == interrupt_handler(EXTI_T->PR1, i, 6))
        {
            bit::set(&(EXTI_T->PR1), i);
        }
    }
}
}

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;
using namespace soc::m4::stm32l4::peripherals;

void EXTI<GPIO>::enable(const Callback& a_callback, uint32_t a_priority)
{
    cml_assert(rcc<mcu>::SYSCFG_mode::enabled == rcc<mcu>::get_syscfg_mode());

    NVIC_SetPriority(static_cast<IRQn_Type>(this->id), a_priority);
    NVIC_EnableIRQ(static_cast<IRQn_Type>(this->id));

    if (this->id >= Id::_0 && this->id <= Id::_4)
    {
        interrupt_handlers[static_cast<uint32_t>(this->id) - static_cast<uint32_t>(Id::_0)].callback = a_callback;
    }
    else
    {
        if (Id::_5_9 == this->id)
        {
            interrupt_handlers[5].callback = a_callback;
        }
        else if (Id::_10_15 == this->id)
        {
            interrupt_handlers[6].callback = a_callback;
        }
    }
}

void EXTI<GPIO>::disable()
{
    NVIC_DisableIRQ(static_cast<IRQn_Type>(this->id));
}

void EXTI<GPIO>::attach(const GPIO& a_port, uint32_t a_pin, Trigger_flag a_trigger)
{
    volatile uint32_t* p_register = &(SYSCFG->EXTICR[a_pin / 4u]);
    uint32_t pos                  = ((static_cast<uint32_t>(a_pin) % 4u) * 4u);

#ifdef CML_ASSERT_ENABLED
    const bool f = bit_flag::is(*p_register, static_cast<uint32_t>(a_port.get_id()) << pos);
    cml_assert((GPIO::Id::a == a_port.get_id() && true == f) || (GPIO::Id::a != a_port.get_id() && false == f));
    cml_assert((Id::_0 == this->id && a_pin == 0u) || (Id::_1 == this->id && a_pin == 1u) ||
               (Id::_2 == this->id && a_pin == 2u) || (Id::_3 == this->id && a_pin == 3u) ||
               (Id::_4 == this->id && a_pin == 4u) || (Id::_5_9 == this->id && (a_pin >= 5u && a_pin <= 9u)) ||
               (Id::_10_15 == this->id && (a_pin >= 10u && a_pin <= 15u)));
#endif

    Interrupt_guard guard;

    bit_flag::set(p_register, 0x3u << pos, static_cast<uint32_t>(a_port.get_id()) << pos);

    bit::clear(&(EXTI_T->RTSR1), a_pin);
    bit::clear(&(EXTI_T->FTSR1), a_pin);
    bit::set(&(EXTI_T->IMR1), a_pin);

    switch (a_trigger)
    {
        case Trigger_flag::rising: {
            bit::set(&(EXTI_T->RTSR1), a_pin);
        }
        break;

        case Trigger_flag::falling: {
            bit::set(&(EXTI_T->FTSR1), a_pin);
        }
        break;

        default: {
            if ((Trigger_flag::rising | Trigger_flag::falling) == a_trigger)
            {
                bit::set(&(EXTI_T->RTSR1), a_pin);
                bit::set(&(EXTI_T->FTSR1), a_pin);
            }
        }
    }
}

void EXTI<GPIO>::deattach(const GPIO& a_port, uint32_t a_pin)
{
    Interrupt_guard guard;

    bit::clear(&(EXTI_T->RTSR1), a_pin);
    bit::clear(&(EXTI_T->FTSR1), a_pin);

    bit_flag::clear(&(SYSCFG->EXTICR[a_pin / 4u]),
                    (static_cast<uint32_t>(a_port.get_id()) << ((static_cast<uint32_t>(a_pin) % 4u) * 4u)));

    interrupt_handlers[a_pin].callback = { nullptr, nullptr };
}

} // namespace stm32l4
} // namespace m4
} // namespace soc