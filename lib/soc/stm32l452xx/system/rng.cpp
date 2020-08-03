/*
    Name: rng.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <soc/stm32l452xx/system/rng.hpp>

//soc
#include <soc/counter.hpp>
#include <soc/Interrupt_guard.hpp>
#include <soc/stm32l452xx/mcu.hpp>

//cml
#include <cml/debug/assert.hpp>
#include <cml/utils/wait.hpp>

namespace {

using namespace soc::stm32l452xx::system;

rng::New_value_callback new_value_callback;

} // namespace ::

extern "C"
{

using namespace cml;
using namespace cml::debug;

void RNG_IRQHandler()
{
    assert(nullptr != new_value_callback.function);

    const uint32_t isr = RNG->SR;
    uint32_t val = 0;

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

namespace soc {
namespace stm32l452xx {
namespace system {

using namespace cml::utils;
using namespace soc::stm32l452xx;

bool rng::enable(uint32_t a_irq_priority, time::tick a_timeout)
{
    assert(mcu::get_clk48_mux_freqency_hz() <= MHz(48));
    assert(a_timeout > 0);

    time::tick start = counter::get();

    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_RNGEN);
    set_flag(&(RNG->CR), RNG_CR_RNGEN);

    bool ret = false == is_flag(RNG->SR, RNG_SR_SEIS);

    if (true == ret)
    {
        ret = wait::until(&(RNG->SR), RNG_SR_SECS, true, start, a_timeout) &&
              wait::until(&(RNG->SR), RNG_SR_CECS, true, start, a_timeout);
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

bool rng::get_value_polling(uint32_t* a_p_value, time::tick a_timeout)
{
    assert(a_timeout > 0);

    time::tick start = counter::get();

    bool ret = wait::until(&(RNG->SR), RNG_SR_DRDY, false, start, a_timeout);

    if (true == ret)
    {
        (*a_p_value) = RNG->DR;
    }

    return ret;
}

void rng::register_new_value_callback(const New_value_callback& a_callback)
{
    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    new_value_callback = a_callback;
    set_flag(&(RNG->CR), RNG_CR_IE);
}

} // namespace system
} // namespace stm32l452xx
} // namespace soc