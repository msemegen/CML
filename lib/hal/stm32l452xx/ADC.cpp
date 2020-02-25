/*
    Name: ADC.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <hal/stm32l452xx/ADC.hpp>

//cml
#include <common/bit.hpp>
#include <debug/assert.hpp>
#include <hal/systick.hpp>
#include <utils/sleep.hpp>

#ifdef CML_DEBUG
#include <hal/stm32l452xx/mcu.hpp>
#endif // CML_DEBUG

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;
using namespace cml::utils;

bool ADC::enable(Resolution a_resolution, const Clock& a_clock, time_tick a_timeout)
{
    assert(true == systick::is_enabled());

    time_tick start = systick::get_counter();

    this->disable();

    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);

    switch (a_clock.source)
    {
        case Clock::Source::asynchronous:
        {
            clear_flag(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE);
            set_flag(&(ADC1_COMMON->CCR), ADC_CCR_PRESC, static_cast<uint32>(a_clock.asynchronous.source));
        }
        break;

        case Clock::Source::synchronous:
        {
            assert(Clock::Synchronous::Divider::_1 == a_clock.synchronous.divider ?
                   mcu::Bus_prescalers::AHB::_1 == mcu::get_bus_prescalers().ahb :
                   true);

            set_flag(&(ADC1_COMMON->CCR), static_cast<uint32>(a_clock.synchronous.divider));
        }
        break;

        case Clock::Source::unknown:
        {
            assert(a_clock.source != Clock::Source::unknown);
        }
    }

    set_flag(&(ADC1->CR), ADC_CR_ADCALDIF);
    sleep::us(21u);

    clear_flag(&(ADC1->CR), ADC_CR_ADCALDIF);
    set_flag(&(ADC1->CR), ADC_CR_ADCAL);

    bool ret = sleep::wait_until(ADC1->CR,
                                 ADC_CR_ADCAL,
                                 true,
                                 systick::get_counter(),
                                 a_timeout - (systick::get_counter() - start));

    if (true == ret)
    {
        set_flag(&(ADC1->CFGR), static_cast<uint32_t>(a_resolution));
        set_flag(&(ADC1->CR), ADC_CR_ADEN);

        ret = sleep::wait_until(ADC1->ISR,
                                ADC_ISR_ADRDY,
                                false,
                                systick::get_counter(),
                                a_timeout - (systick::get_counter() - start));
    }

    if (false == ret)
    {
        this->disable();
    }

    return ret;
}

void ADC::disable()
{
    ADC1->CR         = 0;
    ADC1_COMMON->CCR = 0;

    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml

#endif