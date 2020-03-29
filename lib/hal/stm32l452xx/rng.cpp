/*
    Name: rng.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <hal/stm32l452xx/rng.hpp>

//cml
#include <hal/stm32l452xx/mcu.hpp>
#include <hal/core/systick.hpp>
#include <utils/wait.hpp>

namespace {

using namespace cml::hal::stm32l452xx;

rng::New_value_callback new_value_callback;

} // namespace ::

extern "C"
{

using namespace cml::common;

void RNG_IRQHandler()
{
    assert(nullptr != new_value_callback.function);

    uint32 isr = RNG->SR;
    uint32 val = 0;

    if (true == is_flag(isr, RNG_SR_DRDY))
    {
        val = RNG->DR;
    }

    new_value_callback.function(val,
                                is_flag(isr, RNG_SR_CECS),
                                is_flag(isr, RNG_SR_SECS),
                                new_value_callback.p_user_data);

    NVIC_ClearPendingIRQ(RNG_IRQn);

    clear_flag(&(RNG->CR), RNG_CR_IE);

    new_value_callback = { nullptr, nullptr };
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;
using namespace cml::hal::core;
using namespace cml::utils;

bool rng::enable(uint32 a_irq_priority, time_tick a_timeout_ms)
{
    assert(mcu::get_clk48_mux_freqency_hz() <= MHz(48));
    assert(true == systick::is_enabled());

    time_tick start = systick::get_counter();

    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_RNGEN);
    set_flag(&(RNG->CR), RNG_CR_RNGEN);

    bool ret = false == is_flag(RNG->SR, RNG_SR_SEIS);

    if (true == ret)
    {
        ret = wait::until(&(RNG->SR), RNG_SR_SECS, true, start, a_timeout_ms) &&
              wait::until(&(RNG->SR), RNG_SR_CECS, true, start, a_timeout_ms);
    }

    if (true == ret)
    {
        NVIC_SetPriority(RNG_IRQn, a_irq_priority);
        NVIC_EnableIRQ(RNG_IRQn);
    }

    return ret;
}

void rng::disable()
{
    clear_flag(&(RNG->CR), RNG_CR_RNGEN);
    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_RNGEN);

    NVIC_DisableIRQ(RNG_IRQn);
}

bool rng::get_value_polling(uint32* a_p_value, time_tick a_timeout_ms)
{
    time_tick start = systick::get_counter();

    bool ret = wait::until(&(RNG->SR), RNG_SR_DRDY, false, start, a_timeout_ms);

    if (true == ret)
    {
        (*a_p_value) = RNG->DR;
    }

    return ret;
}

void rng::get_value_it(const New_value_callback& a_callback)
{
    assert(nullptr != a_callback.function);

    new_value_callback = a_callback;
    set_flag(&(RNG->CR), RNG_CR_IE);
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml