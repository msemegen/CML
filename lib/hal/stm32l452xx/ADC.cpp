/*
    Name: ADC.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <hal/stm32l452xx/ADC.hpp>

//cml
#include <utils/sleep.hpp>

#ifdef CML_DEBUG
#include <debug/assert.hpp>
#include <hal/stm32l452xx/mcu.hpp>
#endif // CML_DEBUG

namespace
{

using namespace cml::common;
using namespace cml::hal::stm32l452xx;

ADC* p_adc1 = nullptr;

bool is_channel(ADC::Channel::Id a_type, const ADC::Channel* a_p_channels, uint32 a_channels_count)
{
    bool found = false;

    for (uint32 i = 0; i < a_channels_count && false == found; i++)
    {
        found = a_type == a_p_channels[i].id;
    }

    return found;
}

} // namespace

extern "C"
{

void ADC1_IRQHandler()
{
    assert(nullptr != p_adc1);
    adc_handle_interrupt(p_adc1);
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;
using namespace cml::utils;

bool is_timeout(time_tick a_start, time_tick a_timeout)
{
    return time_tick_infinity == a_timeout ? false : time_tick_diff(systick::get_counter(), a_start) < a_timeout;
}

void adc_handle_interrupt(ADC* a_p_this)
{
    uint32 isr = ADC1->ISR;

    if (true == is_flag(isr, ADC_ISR_EOC))
    {
        bool timeout = is_timeout(a_p_this->callaback.start_timestamp, a_p_this->callaback.timeout);
        bool series_end = is_flag(isr, ADC_ISR_EOS);

        bool ret = a_p_this->callaback.function(ADC1->DR, series_end, timeout);

        if (true == series_end)
        {
            set_flag(&(ADC1->ISR), ADC_ISR_EOS);
        }

        if (true == series_end || true == timeout || false == ret)
        {
            set_flag(&(ADC1->CR), ADC_CR_ADSTP);
            clear_flag(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
        }
    }

    if (true == is_flag(isr, ADC_ISR_OVR))
    {
        isr = isr;
        while (true);
    }
}

bool ADC::enable(Resolution a_resolution, const Asynchronous_clock& a_clock, time_tick a_timeout)
{
    assert(true == systick::is_enabled());
    assert(mcu::Pll_config::Source::unknown != mcu::get_pll_config().source &&
           true == mcu::get_pll_config().pllsai1.r.output_enabled);

    time_tick start = systick::get_counter();

    this->disable();

    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    clear_flag(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE);
    set_flag(&(ADC1_COMMON->CCR), ADC_CCR_PRESC, static_cast<common::uint32>(a_clock.divider));

    return this->enable(a_resolution, start, a_timeout);
}

bool ADC::enable(Resolution a_resolution, const Synchronous_clock& a_clock, time_tick a_timeout)
{
    assert(true == systick::is_enabled());
    assert(Synchronous_clock::Divider::unknown != a_clock.divider);
    assert(Synchronous_clock::Divider::_1 == a_clock.divider ?
           mcu::Bus_prescalers::AHB::_1 == mcu::get_bus_prescalers().ahb :
           true);

    time_tick start = systick::get_counter();

    this->disable();

    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    set_flag(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE, static_cast<common::uint32>(a_clock.divider));

    return this->enable(a_resolution, start, a_timeout);
}

void ADC::disable()
{
    ADC1->CR         = 0;
    ADC1_COMMON->CCR = 0;

    set_flag(&(ADC1->CR), ADC_CR_DEEPPWD);
    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);

    NVIC_DisableIRQ(ADC1_IRQn);

    p_adc1 = nullptr;
}

void ADC::set_active_channels(const Channel* a_p_channels, uint32 a_channels_count)
{
    assert(nullptr != a_p_channels);
    assert(a_channels_count > 0);

    this->clear_active_channels();

    ADC1->SQR1 = a_channels_count - 1;

    for (uint32 i = 0; i < a_channels_count && i < 4; i++)
    {
        assert(Channel::Id::unknown != a_p_channels[i].id);
        set_flag(&(ADC1->SQR1), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32 i = 4; i < a_channels_count && i < 9; i++)
    {
        assert(Channel::Id::unknown != a_p_channels[i].id);
        set_flag(&(ADC1->SQR2), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32 i = 9; i < a_channels_count && i < 14; i++)
    {
        assert(Channel::Id::unknown != a_p_channels[i].id);
        set_flag(&(ADC1->SQR3), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32 i = 14; i < a_channels_count && i < 16; i++)
    {
        assert(Channel::Id::unknown != a_p_channels[i].id);
        set_flag(&(ADC1->SQR4), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    volatile uint32* p_SMPRs = reinterpret_cast<volatile uint32*>(&(ADC1->SMPR1));

    for (uint32 i = 0; i < a_channels_count; i++)
    {
        const uint32 channel_id        = static_cast<uint32_t>(a_p_channels[i].id);
        const uint32 sampling_time_val = static_cast<uint32_t>(a_p_channels[i].sampling_time);
        const uint32 register_index    = channel_id / 10;

        set_flag(&(p_SMPRs[register_index]), sampling_time_val << ((channel_id - (register_index * 10)) * 3));
    }

    bool enable_temperature_sensor = is_channel(Channel::Id::temperature_sensor, a_p_channels, a_channels_count);
    bool enable_voltage_reference  = is_channel(Channel::Id::voltage_reference, a_p_channels, a_channels_count);
    bool enable_battery_voltage    = is_channel(Channel::Id::battery_voltage, a_p_channels, a_channels_count);

    if (true == enable_temperature_sensor)
    {
        set_flag(&(ADC1_COMMON->CCR), ADC_CCR_TSEN);
        sleep::us(120);
    }

    if (true == enable_voltage_reference)
    {
        set_flag(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);
    }

    if (true == enable_battery_voltage)
    {
        set_flag(&(ADC1_COMMON->CCR), ADC_CCR_VBATEN);
    }
}

void ADC::clear_active_channels()
{
    ADC1->SQR1 = 0;

    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->SQR4 = 0;

    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;

    clear_flag(&(ADC1_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);
}

void ADC::read_polling(uint16* a_p_data, uint32 a_count)
{
    assert(nullptr != a_p_data);
    assert(a_count > 0);

    assert(this->get_active_channels_count() == a_count);

    set_flag(&(ADC1->CR), ADC_CR_ADSTART);

    for (uint32 i = 0; i < a_count; i++)
    {
        sleep::until(&(ADC1->ISR), ADC_ISR_EOC, false);
        a_p_data[i] = static_cast<uint16_t>(ADC1->DR);
    }

    sleep::until(&(ADC1->ISR), ADC_ISR_EOS, false);
    set_flag(&(ADC1->ISR), ADC_ISR_EOS);

    set_flag(&(ADC1->CR), ADC_CR_ADSTP);
    clear_flag(&(ADC1->CR), ADC_CR_ADSTART);
}

bool ADC::read_polling(uint16* a_p_data, uint32 a_count, time_tick a_timeout)
{
    assert(nullptr != a_p_data);
    assert(a_count > 0);

    assert(this->get_active_channels_count() == a_count);
    assert(true == systick::is_enabled());

    set_flag(&(ADC1->CR), ADC_CR_ADSTART);

    bool ret        = true;
    time_tick start = systick::get_counter();

    for (uint32 i = 0; i < a_count && true == ret; i++)
    {
        ret = sleep::until(&(ADC1->ISR), ADC_ISR_EOC, false, start, a_timeout);

        if (true == ret)
        {
            a_p_data[i] = static_cast<uint16_t>(ADC1->DR);
        }
    }

    if (true == ret)
    {
        ret = sleep::until(&(ADC1->ISR), ADC_ISR_EOS, false, start, a_timeout);

        if (true == ret)
        {
            set_flag(&(ADC1->ISR), ADC_ISR_EOS);
        }
    }

    set_flag(&(ADC1->CR), ADC_CR_ADSTP);
    clear_flag(&(ADC1->CR), ADC_CR_ADSTART);

    return ret;
}

void ADC::read_it(Conversion_callback a_callback, time_tick a_timeout)
{
    assert(true == systick::is_enabled());

    if (nullptr != a_callback)
    {
        this->callaback.function = a_callback;

        this->callaback.timeout         = a_timeout;
        this->callaback.start_timestamp = systick::get_counter();

        set_flag(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
        set_flag(&(ADC1->CR), ADC_CR_ADSTART);
    }
    else
    {
        this->callaback.function = nullptr;
        this->callaback.timeout = time_tick_infinity;
    }
}

bool ADC::enable(Resolution a_resolution, time_tick a_start, time_tick a_timeout)
{
    p_adc1 = this;

    NVIC_SetPriority(ADC1_IRQn, config::adc::_1_interrupt_priority);
    NVIC_EnableIRQ(ADC1_IRQn);

    clear_flag(&(ADC1->CR), ADC_CR_DEEPPWD);
    set_flag(&(ADC1->CR), ADC_CR_ADVREGEN);
    sleep::us(21u);

    clear_flag(&(ADC1->CR), ADC_CR_ADCALDIF);
    set_flag(&(ADC1->CR), ADC_CR_ADCAL);

    bool ret = sleep::until(&(ADC1->CR), ADC_CR_ADCAL, true, a_start, a_timeout);

    if (true == ret)
    {
        set_flag(&(ADC1->CFGR), static_cast<uint32_t>(a_resolution));
        set_flag(&(ADC1->CR), ADC_CR_ADEN);

        ret = sleep::until(&(ADC1->ISR), ADC_ISR_ADRDY, false, a_start, a_timeout);
    }

    if (true == ret)
    {
        set_flag(&(ADC1->ISR), ADC_ISR_ADRDY);
    }

    if (false == ret)
    {
        this->disable();
    }

    return ret;
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml

#endif // STM32L452xx