/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/RNG/Interrupt.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {
using namespace soc::m4::stm32l4;

Interrupt<RNG>* irq_context[1] = { nullptr };
} // namespace

// SIC! GCC bug?
#ifdef __GNUC__
namespace soc {
namespace m4 {
namespace stm32l4 {
extern void RNG_interrupt_handler();
} // namespace stm32l4
} // namespace m4
} // namespace soc
#endif

extern "C" {

using namespace soc::m4::stm32l4;

void RNG_IRQHandler()
{
    RNG_interrupt_handler();
}
}

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

#define RNG_T ((RNG_TypeDef*)RNG_BASE)
Interrupt<RNG>::Callback Interrupt<RNG>::callback;

void RNG_interrupt_handler()
{
    cml_assert(nullptr != irq_context[0]);

    const std::uint32_t isr = RNG_T->SR;
    std::uint32_t value     = 0;

    if (true == bit_flag::is(isr, RNG_SR_DRDY))
    {
        value = RNG_T->DR;
    }

    if (nullptr != irq_context[0]->callback.function)
    {
        irq_context[0]->callback.function(value,
                                          bit_flag::is(isr, RNG_SR_CECS),
                                          bit_flag::is(isr, RNG_SR_SECS),
                                          Interrupt<RNG>::callback.p_user_data);
    }

    NVIC_ClearPendingIRQ(RNG_IRQn);
}

void Interrupt<RNG>::enable(const IRQ_config& a_irq_config)
{
    cml_assert(nullptr == irq_context[0]);

    irq_context[0] = this;

    NVIC_SetPriority(
        IRQn_Type::RNG_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(IRQn_Type::RNG_IRQn);
}

void Interrupt<RNG>::disable()
{
    NVIC_DisableIRQ(IRQn_Type::RNG_IRQn);

    irq_context[0] = nullptr;
}

void Interrupt<RNG>::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    Interrupt<RNG>::callback = a_callback;
    bit_flag::set(&(RNG_T->CR), RNG_CR_IE);
}

void Interrupt<RNG>::unregister_callback()
{
    bit_flag::clear(&(RNG_T->CR), RNG_CR_IE);
    Interrupt<RNG>::callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif