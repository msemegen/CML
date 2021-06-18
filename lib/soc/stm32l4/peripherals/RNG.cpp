/*
 *   Name: RNG.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/stm32l4/peripherals/RNG.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/stm32l4/mcu.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>

namespace {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

using namespace soc::stm32l4::peripherals;

RNG::New_value_callback new_value_callback;

bool created = false;

#endif

} // namespace

extern "C" {

using namespace cml;
using namespace cml::debug;

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

#define RNG_T ((RNG_TypeDef*)RNG_BASE)

void RNG_IRQHandler()
{
    cml_assert(nullptr != new_value_callback.function);

    const uint32_t isr = RNG_T->SR;
    uint32_t value     = 0;

    if (true == bit_flag::is(isr, RNG_SR_DRDY))
    {
        value = RNG_T->DR;
    }

    new_value_callback.function(
        value, bit_flag::is(isr, RNG_SR_CECS), bit_flag::is(isr, RNG_SR_SECS), new_value_callback.p_user_data);

    NVIC_ClearPendingIRQ(RNG_IRQn);
}

#endif

} // extern "C"

namespace soc {
namespace stm32l4 {
namespace peripherals {

using namespace cml::utils;

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

RNG::RNG()
{
    cml_assert(false == created);
    created = true;
}

RNG::~RNG()
{
    this->disable();
    created = false;
}

bool RNG::enable(uint32_t a_irq_priority, uint32_t a_timeout)
{
    cml_assert(mcu::get_clk48_mux_freqency_hz() <= 48 * 1000000u);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    bit_flag::set(&(RNG_T->CR), RNG_CR_RNGEN);

    bool ret = false == bit_flag::is(RNG_T->SR, RNG_SR_SEIS);

    if (true == ret)
    {
        ret = wait_until::all_bits(&(RNG_T->SR), RNG_SR_SECS, true, start, a_timeout) &&
              wait_until::all_bits(&(RNG_T->SR), RNG_SR_CECS, true, start, a_timeout);
    }

    if (true == ret)
    {
        NVIC_SetPriority(RNG_IRQn, a_irq_priority);
        NVIC_EnableIRQ(RNG_IRQn);
    }

    return ret;
}

void RNG::disable()
{
    bit_flag::clear(&(RNG_T->CR), RNG_CR_RNGEN);
    NVIC_DisableIRQ(RNG_IRQn);
}

bool RNG::get_value_polling(uint32_t* a_p_value, uint32_t a_timeout)
{
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    bool ret = wait_until::all_bits(&(RNG_T->SR), RNG_SR_DRDY, false, start, a_timeout);

    if (true == ret)
    {
        (*a_p_value) = RNG_T->DR;
    }

    return ret;
}

void RNG::register_new_value_callback(const New_value_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    new_value_callback = a_callback;

    bit_flag::set(&(RNG_T->CR), RNG_CR_IE);
}

void RNG::unregister_new_value_callback()
{
    cml_assert(nullptr != new_value_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(RNG_T->CR), RNG_CR_IE);

    new_value_callback = { nullptr, nullptr };
}

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc

namespace soc {
namespace stm32l4 {

using namespace soc::stm32l4::peripherals;

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
} // namespace soc

#endif // STM32L4