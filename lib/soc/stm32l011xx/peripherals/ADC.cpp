/*
    Name: ADC.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/peripherals/ADC.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/system_timer.hpp>
#include <soc/stm32l011xx/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assert.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/wait.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l011xx::peripherals;

ADC* p_adc_1 = nullptr;
ADC::Conversion_callback callaback;

bool is_channel(ADC::Channel a_type, const ADC::Channel* a_p_channels, uint32_t a_channels_count)
{
    bool found = false;

    for (uint32_t i = 0; i < a_channels_count && false == found; i++)
    {
        found = a_type == a_p_channels[i];
    }

    return found;
}

} // namespace

extern "C" {

void ADC1_COMP_IRQHandler()
{
    assert(nullptr != p_adc_1);
    adc_interrupt_handler(p_adc_1);
}

} // extern "C"

namespace soc {
namespace stm32l011xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void adc_interrupt_handler(ADC* a_p_this)
{
    const uint32_t isr = ADC1->ISR;

    if (true == bit_flag::is(isr, ADC_ISR_EOC))
    {
        const bool series_end = bit_flag::is(isr, ADC_ISR_EOS);
        const bool ret        = callaback.function(ADC1->DR, series_end, callaback.p_user_data);

        if (true == series_end)
        {
            bit_flag::set(&(ADC1->ISR), ADC_ISR_EOS);
        }

        if (true == series_end || false == ret)
        {
            bit_flag::set(&(ADC1->CR), ADC_CR_ADSTP);
            bit_flag::clear(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
        }
    }
}

bool ADC::enable(Resolution a_resolution,
                 const Synchronous_clock& a_clock,
                 uint32_t a_irq_priority,
                 time::tick a_timeout)
{
    assert(nullptr == p_adc_1);
    assert(a_timeout > 0);

    time::tick start = system_timer::get();

    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_ADC1EN);
    bit_flag::set(&(ADC1->CFGR2), ADC_CFGR2_CKMODE, static_cast<uint32_t>(a_clock.divider));

    return this->enable(a_resolution, start, a_irq_priority, a_timeout);
}

bool ADC::enable(Resolution a_resolution,
                 const Asynchronous_clock& a_clock,
                 uint32_t a_irq_priority,
                 time::tick a_timeout)
{
    assert(nullptr == p_adc_1);
    assert(true == mcu::is_clock_enabled(mcu::Clock::hsi));

    time::tick start = system_timer::get();

    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_ADC1EN);
    bit_flag::set(&(ADC1->CFGR2), ADC_CFGR2_CKMODE, static_cast<uint32_t>(a_clock.divider));

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

    bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_ADC1EN);

    p_adc_1 = nullptr;
}

void ADC::set_active_channels(Sampling_time a_sampling_time, const Channel* a_p_channels, uint32_t a_channels_count)
{
    assert(nullptr != p_adc_1);

    ADC1->SMPR = static_cast<uint32_t>(a_sampling_time);

    for (uint32_t i = 0; i < a_channels_count; i++)
    {
        bit::set(&(ADC1->CHSELR), static_cast<uint32_t>(a_p_channels[i]));
    }

    bool is_temperature_sensor = is_channel(Channel::temperature_sensor, a_p_channels, a_channels_count);
    bool is_voltage_reference  = is_channel(Channel::voltage_reference, a_p_channels, a_channels_count);

    if (true == is_temperature_sensor)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_TSEN);
        delay::us(10);
    }

    if (true == is_voltage_reference)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);
    }
}

void ADC::clear_active_channels()
{
    assert(nullptr != p_adc_1);

    ADC1->SMPR   = 0;
    ADC1->CHSELR = 0;

    bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN);
}

void ADC::read_polling(uint16_t* a_p_data, uint32_t a_count)
{
    assert(nullptr != p_adc_1);
    assert(nullptr != a_p_data);
    assert(a_count > 0);

    assert(this->get_active_channels_count() == a_count);

    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTART);

    for (uint32_t i = 0; i < a_count; i++)
    {
        wait::until(&(ADC1->ISR), ADC_ISR_EOC, false);
        a_p_data[i] = static_cast<uint16_t>(ADC1->DR);
    }

    wait::until(&(ADC1->ISR), ADC_ISR_EOS, false);
    bit_flag::set(&(ADC1->ISR), ADC_ISR_EOS);

    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(ADC1->CR), ADC_CR_ADSTART);
}

bool ADC::read_polling(uint16_t* a_p_data, uint32_t a_count, time::tick a_timeout)
{
    assert(nullptr != p_adc_1);
    assert(nullptr != a_p_data);
    assert(a_count > 0);
    assert(a_timeout > 0);
    assert(this->get_active_channels_count() == a_count);

    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTART);

    bool ret         = true;
    time::tick start = system_timer::get();

    for (uint32_t i = 0; i < a_count && true == ret; i++)
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
            bit_flag::set(&(ADC1->ISR), ADC_ISR_EOS);
        }
    }

    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(ADC1->CR), ADC_CR_ADSTART);

    return ret;
}

void ADC::register_conversion_callback(const Conversion_callback& a_callback)
{
    assert(nullptr != p_adc_1);
    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    callaback = a_callback;

    bit_flag::set(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTART);
}

void ADC::unregister_conversion_callback()
{
    assert(nullptr != p_adc_1);

    Interrupt_guard guard;

    bit_flag::clear(&(ADC1->CR), ADC_CR_ADSTART);
    bit_flag::clear(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);

    callaback = { nullptr, nullptr };
}

void ADC::set_resolution(Resolution a_resolution)
{
    assert(nullptr != p_adc_1);

    bool is_started = bit_flag::is(ADC1->CR, ADC_CR_ADSTART);

    if (true == is_started)
    {
        bit_flag::clear(&(ADC1->CR), ADC_CR_ADSTART);
    }

    bit_flag::set(&(ADC1->CFGR1), static_cast<uint32_t>(a_resolution));

    if (true == is_started)
    {
        bit_flag::set(&(ADC1->CR), ADC_CR_ADSTART);
    }
}

uint32_t ADC::get_active_channels_count() const
{
    assert(nullptr != p_adc_1);

    uint32_t ret = 0;

    for (uint32_t i = 0; i <= 18; i++)
    {
        ret += true == bit::is(ADC1->CHSELR, i) ? 1 : 0;
    }

    return ret;
}

bool ADC::enable(Resolution a_resolution, time::tick a_start, uint32_t a_irq_priority, time::tick a_timeout)
{
    p_adc_1 = this;

    NVIC_SetPriority(ADC1_COMP_IRQn, a_irq_priority);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);

    if (mcu::get_sysclk_frequency_hz() < 3500000)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_LFMEN);
    }

    bit_flag::set(&(ADC1->CR), ADC_CR_ADVREGEN);
    delay::us(2);

    bit_flag::set(&(ADC1->CR), ADC_CR_ADCAL);

    bool ret = wait::until(&(ADC1->CR), ADC_CR_ADCAL, true, a_start, a_timeout);

    if (true == ret)
    {
        bit_flag::set(&(ADC1->CFGR1), static_cast<uint32_t>(a_resolution));
        bit_flag::set(&(ADC1->CR), ADC_CR_ADEN);

        ret = wait::until(&(ADC1->CR), ADC_CR_ADEN, false, a_start, a_timeout);

        if (true == ret)
        {
            bit_flag::set(&(ADC1->ISR), ADC_ISR_ADRDY);
        }
    }

    if (false == ret)
    {
        this->disable();
    }

    return ret;
}

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx