/*
 *   Name: RNG.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/stm32l452xx/peripherals/RNG.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/stm32l452xx/mcu.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>

namespace {

using namespace soc::stm32l452xx::peripherals;

RNG::New_value_callback new_value_callback;

} // namespace

extern "C" {

using namespace cml;
using namespace cml::debug;

void RNG_IRQHandler()
{
    cml_assert(nullptr != new_value_callback.function);

#ifndef RNG
#define RNG ((RNG_TypeDef*)RNG_BASE)
#endif

    const uint32_t isr = RNG->SR;
    uint32_t value     = 0;

    if (true == bit_flag::is(isr, RNG_SR_DRDY))
    {
        value = RNG->DR;
    }

#ifdef RNG
#undef RNG
#endif

    new_value_callback.function(
        value, bit_flag::is(isr, RNG_SR_CECS), bit_flag::is(isr, RNG_SR_SECS), new_value_callback.p_user_data);

    NVIC_ClearPendingIRQ(RNG_IRQn);
}

} // extern "C"

namespace soc {
namespace stm32l452xx {
namespace peripherals {

using namespace cml::utils;
using namespace soc::stm32l452xx;

RNG::RNG() {}

RNG::~RNG() {}

bool RNG::enable(uint32_t a_irq_priority, uint32_t a_timeout)
{
    cml_assert(mcu::get_clk48_mux_freqency_hz() <= 48 * 1000000u);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

#ifndef RNG
#define RNG ((RNG_TypeDef*)RNG_BASE)
#endif

    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_RNGEN);
    bit_flag::set(&(RNG->CR), RNG_CR_RNGEN);

    bool ret = false == bit_flag::is(RNG->SR, RNG_SR_SEIS);

    if (true == ret)
    {
        ret = wait_until::all_bits(&(RNG->SR), RNG_SR_SECS, true, start, a_timeout) &&
              wait_until::all_bits(&(RNG->SR), RNG_SR_CECS, true, start, a_timeout);
    }

#ifdef RNG
#undef RNG
#endif

    if (true == ret)
    {
        NVIC_SetPriority(RNG_IRQn, a_irq_priority);
        NVIC_EnableIRQ(RNG_IRQn);
    }

    return ret;
}

void RNG::disable()
{
#ifndef RNG
#define RNG ((RNG_TypeDef*)RNG_BASE)
#endif

    bit_flag::clear(&(RNG->CR), RNG_CR_RNGEN);
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_RNGEN);

#ifdef RNG
#undef RNG
#endif

    NVIC_DisableIRQ(RNG_IRQn);
}

bool RNG::get_value_polling(uint32_t* a_p_value, uint32_t a_timeout)
{
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

#ifndef RNG
#define RNG ((RNG_TypeDef*)RNG_BASE)
#endif

    bool ret = wait_until::all_bits(&(RNG->SR), RNG_SR_DRDY, false, start, a_timeout);

    if (true == ret)
    {
        (*a_p_value) = RNG->DR;
    }

#ifdef RNG
#undef RNG
#endif

    return ret;
}

void RNG::register_new_value_callback(const New_value_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    new_value_callback = a_callback;

#ifndef RNG
#define RNG ((RNG_TypeDef*)RNG_BASE)
#endif

    bit_flag::set(&(RNG->CR), RNG_CR_IE);

#ifdef RNG
#undef RNG
#endif
}

void RNG::unregister_new_value_callback()
{
    cml_assert(nullptr != new_value_callback.function);

    Interrupt_guard guard;

#ifndef RNG
#define RNG ((RNG_TypeDef*)RNG_BASE)
#endif

    bit_flag::clear(&(RNG->CR), RNG_CR_IE);

#ifdef RNG
#undef RNG
#endif

    new_value_callback = { nullptr, nullptr };
}

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc