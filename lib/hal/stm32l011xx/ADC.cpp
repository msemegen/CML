/*
    Name: ADC.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

//this
#include <hal/stm32l011xx/ADC.hpp>

//cml
#include <common/bit.hpp>
#include <debug/assert.hpp>
#include <hal/systick.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

using namespace cml::common;

bool ADC::enable(Resolution a_resolution, const Synchronous_clock& a_clock, time_tick a_timeout)
{
    assert(true == systick::is_enabled());

    time_tick start = systick::get_counter();

    this->disable();

    set_flag(&(RCC->APB2ENR), RCC_APB2ENR_ADC1EN);
    set_flag(&(ADC1->CFGR2), ADC_CFGR2_CKMODE, static_cast<uint32>(a_clock.divider));


    return this->enable(a_resolution, start, a_timeout);
}

bool ADC::enable(Resolution a_resolution, const Asynchronous_clock& a_clock, time_tick a_timeout)
{
    assert(true == systick::is_enabled());

    time_tick start = systick::get_counter();

    this->disable();

    set_flag(&(RCC->APB2ENR), RCC_APB2ENR_ADC1EN);
    set_flag(&(ADC1->CFGR2), ADC_CFGR2_CKMODE, static_cast<uint32>(a_clock.divider));

    return this->enable(a_resolution, start, a_timeout);
}

void ADC::disable()
{

    NVIC_DisableIRQ(ADC1_COMP_IRQn);
}

bool ADC::enable(Resolution a_resolution, time_tick a_start, time_tick a_timeout)
{
    NVIC_SetPriority(ADC1_COMP_IRQn, 0);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);


}

} // namespace cml
} // namespace hal
} // namespace stm32l011xx

#endif // STM32L011xx