
//this
#include <hal/stm32l452xx/rng.hpp>

//cml
#include <hal/stm32l452xx/mcu.hpp>
#include <hal/systick.hpp>
#include <utils/sleep.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;
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
        ret = sleep::until(&(RNG->SR), RNG_SR_SECS , true, start, a_timeout_ms) &&
              sleep::until(&(RNG->SR), RNG_SR_CECS, true, start, a_timeout_ms);
        ;
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

    bool ret = sleep::until(&(RNG->SR), RNG_SR_DRDY, false, start, a_timeout_ms);

    if (true == ret)
    {
        (*a_p_value) = RNG->DR;
    }

    return ret;
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml