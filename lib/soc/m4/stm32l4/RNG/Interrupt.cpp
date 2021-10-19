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

namespace {
using namespace soc::m4::stm32l4;

Interrupt<RNG>* p_RNGs = nullptr;
#define RNG_T ((RNG_TypeDef*)RNG_BASE)

} // namespace

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

void RNG_interrupt_handler()
{
    cml_assert(nullptr != p_RNGs);

    const std::uint32_t isr = RNG_T->SR;
    std::uint32_t value     = 0;

    if (true == bit_flag::is(isr, RNG_SR_DRDY))
    {
        value = RNG_T->DR;
    }

    p_RNGs->new_value_callback.function(value,
                                        bit_flag::is(isr, RNG_SR_CECS),
                                        bit_flag::is(isr, RNG_SR_SECS),
                                        p_RNGs->p_RNG,
                                        p_RNGs->new_value_callback.p_user_data);

    NVIC_ClearPendingIRQ(RNG_IRQn);
}

Interrupt<RNG>::Interrupt(RNG* a_p_RNG)
    : p_RNG(a_p_RNG)
{
    cml_assert(nullptr == p_RNG);
    p_RNGs = this;
}

Interrupt<RNG>::~Interrupt()
{
    p_RNGs = nullptr;
}

void Interrupt<RNG>::enable(const IRQ& a_irq)
{
    cml_assert(true == a_irq.active);

    NVIC_SetPriority(IRQn_Type::RNG_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq.preempt_priority, a_irq.sub_priority));
    NVIC_EnableIRQ(IRQn_Type::RNG_IRQn);
}

void Interrupt<RNG>::disable()
{
    NVIC_DisableIRQ(IRQn_Type::RNG_IRQn);
}

void Interrupt<RNG>::register_callback(const New_value_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->new_value_callback = a_callback;
        bit_flag::set(&(RNG_T->CR), RNG_CR_IE);
    }
    else
    {
        bit_flag::clear(&(RNG_T->CR), RNG_CR_IE);
        this->new_value_callback = { nullptr, nullptr };
    }
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif