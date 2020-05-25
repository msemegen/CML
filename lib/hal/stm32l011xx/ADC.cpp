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
#include <hal/mcu.hpp>
#include <hal/core/systick.hpp>
#include <utils/delay.hpp>
#include <utils/wait.hpp>

namespace {

using namespace cml::common;
using namespace cml::hal::stm32l011xx;

ADC* p_adc_1 = nullptr;
ADC::Conversion_callback callaback;

bool is_channel(ADC::Channel a_type, const ADC::Channel* a_p_channels, uint32 a_channels_count)
{
    bool found = false;

    for (uint32 i = 0; i < a_channels_count && false == found; i++)
    {
        found = a_type == a_p_channels[i];
    }

    return found;
}

} // namespace ::

extern "C"
{

void ADC1_COMP_IRQHandler()
{
    assert(nullptr != p_adc_1);
    adc_handle_interrupt(p_adc_1);
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l011xx {

using namespace cml::common;
using namespace cml::hal::core;
using namespace cml::utils;

void adc_handle_interrupt(ADC* a_p_this)
{
    uint32 isr = ADC1->ISR;

    if (true == is_flag(isr, ADC_ISR_EOC))
    {
        const bool series_end = is_flag(isr, ADC_ISR_EOS);
        const bool ret = callaback.function(ADC1->DR, series_end, callaback.p_user_data);

        if (true == series_end)
        {
            set_flag(&(ADC1->ISR), ADC_ISR_EOS);
        }

        if (true == series_end || false == ret)
        {
            set_flag(&(ADC1->CR), ADC_CR_ADSTP);
            clear_flag(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
        }
    }
}

bool ADC::enable(Resolution a_resolution, const Synchronous_clock& a_clock, uint32 a_irq_priority, time::tick a_timeout)
{
    assert(nullptr == p_adc_1);
    assert(true == systick::is_enabled());

    time::tick start = systick::get_counter();

    set_flag(&(RCC->APB2ENR), RCC_APB2ENR_ADC1EN);
    set_flag(&(ADC1->CFGR2), ADC_CFGR2_CKMODE, static_cast<uint32>(a_clock.divider));

    return this->enable(a_resolution, start, a_irq_priority, a_timeout);
}

bool ADC::enable(Resolution a_resolution, const Asynchronous_clock& a_clock, uint32 a_irq_priority, time::tick a_timeout)
{
    assert(nullptr == p_adc_1);
    assert(true == systick::is_enabled());
    assert(true == mcu::is_clock_enabled(mcu::Clock::hsi));

    time::tick start = systick::get_counter();

    set_flag(&(RCC->APB2ENR), RCC_APB2ENR_ADC1EN);
    set_flag(&(ADC1->CFGR2), ADC_CFGR2_CKMODE, static_cast<uint32>(a_clock.divider));

    return this->enable(a_resolution, start, a_irq_priority, a_timeout);
}

void ADC::disable()
{
    assert(nullptr != p_adc_1);

    ADC1->CR         = 0;
    ADC1->CFGR1      = 0;
    ADC1->CFGR2      = 0;
    ADC1_COMMON->CCR = 0;

    NVIC_DisableIRQ(ADC1_COMP_IRQn);

    clear_flag(&(RCC->APB2ENR), RCC_APB2ENR_ADC1EN);

    p_adc_1 = nullptr;
}

void ADC::set_active_channels(Sampling_time a_sampling_time, const Channel* a_p_channels, uint32 a_channels_count)
{
    assert(nullptr != p_adc_1);

    ADC1->SMPR = static_cast<uint32_t>(a_sampling_time);

    for (uint32 i = 0; i < a_channels_count; i++)
    {
        set_bit(&(ADC1->CHSELR), static_cast<uint32>(a_p_channels[i]));
    }

    bool is_temperature_sensor = is_channel(Channel::temperature_sensor, a_p_channels, a_channels_count);
    bool is_voltage_reference  = is_channel(Channel::voltage_reference, a_p_channels, a_channels_count);

    if (true == is_temperature_sensor)
    {
        set_flag(&(ADC1_COMMON->CCR), ADC_CCR_TSEN);
        delay::us(10);
    }

    if (true == is_voltage_reference)
    {
        set_flag(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);
    }
}

void ADC::clear_active_channels()
{
    assert(nullptr != p_adc_1);

    ADC1->SMPR   = 0;
    ADC1->CHSELR = 0;

    clear_flag(&(ADC1_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN);
}

void ADC::read_polling(uint16* a_p_data, uint32 a_count)
{
    assert(nullptr != p_adc_1);
    assert(nullptr != a_p_data);
    assert(a_count > 0);

    assert(this->get_active_channels_count() == a_count);

    set_flag(&(ADC1->CR), ADC_CR_ADSTART);

    for (uint32 i = 0; i < a_count; i++)
    {
        wait::until(&(ADC1->ISR), ADC_ISR_EOC, false);
        a_p_data[i] = static_cast<uint16_t>(ADC1->DR);
    }

    wait::until(&(ADC1->ISR), ADC_ISR_EOS, false);
    set_flag(&(ADC1->ISR), ADC_ISR_EOS);

    set_flag(&(ADC1->CR), ADC_CR_ADSTP);
    clear_flag(&(ADC1->CR), ADC_CR_ADSTART);
}

bool ADC::read_polling(uint16* a_p_data, uint32 a_count, time::tick a_timeout)
{
    assert(nullptr != p_adc_1);
    assert(nullptr != a_p_data);
    assert(a_count > 0);

    assert(this->get_active_channels_count() == a_count);
    assert(true == systick::is_enabled());

    set_flag(&(ADC1->CR), ADC_CR_ADSTART);

    bool ret = true;
    time::tick start = systick::get_counter();

    for (uint32 i = 0; i < a_count && true == ret; i++)
    {
        ret = wait::until(&(ADC1->ISR), ADC_ISR_EOC, false, start, a_timeout);

        if (true == ret)
        {
            a_p_data[i] = static_cast<uint16_t>(ADC1->DR);
        }
    }

    if (true == ret)
    {
        ret = wait::until(&(ADC1->ISR), ADC_ISR_EOS, false, start, a_timeout);

        if (true == ret)
        {
            set_flag(&(ADC1->ISR), ADC_ISR_EOS);
        }
    }

    set_flag(&(ADC1->CR), ADC_CR_ADSTP);
    clear_flag(&(ADC1->CR), ADC_CR_ADSTART);

    return ret;
}

void ADC::start_read_it(const Conversion_callback& a_callback)
{
    assert(nullptr != p_adc_1);
    assert(nullptr != a_callback.function);

    callaback = a_callback;

    set_flag(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    set_flag(&(ADC1->CR), ADC_CR_ADSTART);
}

void ADC::stop_read_it()
{
    assert(nullptr != p_adc_1);

    clear_flag(&(ADC1->CR), ADC_CR_ADSTART);
    clear_flag(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);

    callaback = { nullptr, nullptr };
}

void ADC::set_resolution(Resolution a_resolution)
{
    assert(nullptr != p_adc_1);

    bool is_started = is_flag(ADC1->CR, ADC_CR_ADSTART);

    if (true == is_started)
    {
        clear_flag(&(ADC1->CR), ADC_CR_ADSTART);
    }

    set_flag(&(ADC1->CFGR1), static_cast<uint32>(a_resolution));

    if (true == is_started)
    {
        set_flag(&(ADC1->CR), ADC_CR_ADSTART);
    }
}

uint32 ADC::get_active_channels_count() const
{
    assert(nullptr != p_adc_1);

    uint32 ret = 0;

    for (uint32 i = 0; i <= 18; i++)
    {
        ret += true == get_bit(ADC1->CHSELR, i) ? 1 : 0;
    }

    return ret;
}

bool ADC::enable(Resolution a_resolution, time::tick a_start, uint32 a_irq_priority, time::tick a_timeout)
{
    p_adc_1 = this;

    NVIC_SetPriority(ADC1_COMP_IRQn, a_irq_priority);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);

    if (mcu::get_sysclk_frequency_hz() < 3500000)
    {
        set_flag(&(ADC1_COMMON->CCR), ADC_CCR_LFMEN);
    }

    set_flag(&(ADC1->CR), ADC_CR_ADVREGEN);
    delay::us(2);

    set_flag(&(ADC1->CR), ADC_CR_ADCAL);

    bool ret = wait::until(&(ADC1->CR), ADC_CR_ADCAL, true, a_start, a_timeout);

    if (true == ret)
    {
        set_flag(&(ADC1->CFGR1), static_cast<uint32>(a_resolution));
        set_flag(&(ADC1->CR), ADC_CR_ADEN);

        ret = wait::until(&(ADC1->CR), ADC_CR_ADEN, false, a_start, a_timeout);

        if (true == ret)
        {
            set_flag(&(ADC1->ISR), ADC_ISR_ADRDY);
        }
    }

    if (false == ret)
    {
        this->disable();
    }

    return ret;
}

} // namespace cml
} // namespace hal
} // namespace stm32l011xx

#endif // STM32L011xx