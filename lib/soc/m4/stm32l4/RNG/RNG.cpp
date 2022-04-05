/*
 *   Name: RNG.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/RNG/RNG.hpp>

// soc
#include <soc/m4/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/mcu/mcu.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/tick_counter.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace {
using namespace soc::m4::stm32l4;

RNG* irq_context[1] = { nullptr };
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
using namespace cml::utils;

void RNG_interrupt_handler()
{
    cml_assert(nullptr != irq_context[0]);

    const std::uint32_t isr = RNG_T->SR;
    std::uint32_t value     = 0;

    if (true == bit_flag::is(isr, RNG_SR_DRDY))
    {
        value = RNG_T->DR;
    }

    if (nullptr != irq_context[0]->interrupt.callback.function)
    {
        irq_context[0]->interrupt.callback.function(value,
                                                    bit_flag::is(isr, RNG_SR_CECS),
                                                    bit_flag::is(isr, RNG_SR_SECS),
                                                    irq_context[0]->interrupt.callback.p_user_data);
    }

    NVIC_ClearPendingIRQ(RNG_IRQn);
}

bool RNG::enable(Milliseconds a_timeout)
{
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);
    cml_assert(rcc<mcu>::get_CLK48_frequency_Hz() <= 48_MHz);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    bit_flag::set(&(RNG_T->CR), RNG_CR_RNGEN);

    bool ret = false == bit_flag::is(RNG_T->SR, RNG_SR_SEIS);

    if (true == ret)
    {
        ret = wait_until::all_bits(&(RNG_T->SR), RNG_SR_SECS, true, start, a_timeout) &&
              wait_until::all_bits(&(RNG_T->SR), RNG_SR_CECS, true, start, a_timeout);
    }

    return ret;
}

void RNG::disable()
{
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    bit_flag::clear(&(RNG_T->CR), RNG_CR_RNGEN);
    NVIC_DisableIRQ(RNG_IRQn);
}

bool RNG::Polling::get_value(std::uint32_t* a_p_value, Milliseconds a_timeout)
{
    cml_assert(true == this->is_created());

    cml_assert(a_timeout > 0_ms);

    bool ret = wait_until::all_bits(&(RNG_T->SR), RNG_SR_DRDY, false, tick_counter::get(), a_timeout);

    if (true == ret)
    {
        (*a_p_value) = RNG_T->DR;
    }

    return ret;
}

void RNG::Interrupt::enable(const IRQ_config& a_irq_config)
{
    cml_assert(true == this->is_created());

    cml_assert(nullptr == irq_context[0]);

    irq_context[0] = this->p_RNG;

    NVIC_SetPriority(
        IRQn_Type::RNG_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(IRQn_Type::RNG_IRQn);
}

void RNG::Interrupt::disable()
{
    cml_assert(true == this->is_created());

    NVIC_DisableIRQ(IRQn_Type::RNG_IRQn);
    this->unregister_callback();

    irq_context[0] = nullptr;
}

void RNG::Interrupt::register_callback(const Callback& a_callback)
{
    cml_assert(true == this->is_created());

    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    bit_flag::set(&(RNG_T->CR), RNG_CR_IE);
}

void RNG::Interrupt::unregister_callback()
{
    cml_assert(true == this->is_created());

    bit_flag::clear(&(RNG_T->CR), RNG_CR_IE);
    this->callback = { nullptr, nullptr };
}

void rcc<RNG>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_RNGEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_RNGSMEN);
    }
}
void rcc<RNG>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_RNGEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_RNGSMEN);
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4