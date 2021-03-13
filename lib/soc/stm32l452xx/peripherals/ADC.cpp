/*
 *   Name: ADC.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/peripherals/ADC.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/wait_until.hpp>

#ifdef CML_ASSERT
#include <soc/stm32l452xx/mcu.hpp>
#endif // CML_ASSERT

namespace {

using namespace cml;
using namespace soc::stm32l452xx::peripherals;

ADC* p_adc_1 = nullptr;

bool is_channel(ADC::Channel::Id a_type, const ADC::Channel* a_p_channels, uint32_t a_channels_count)
{
    bool found = false;

    for (uint32_t i = 0; i < a_channels_count && false == found; i++)
    {
        found = a_type == a_p_channels[i].id;
    }

    return found;
}

} // namespace

extern "C" {

void ADC1_IRQHandler()
{
    cml_assert(nullptr != p_adc_1);
    adc_interrupt_handler(p_adc_1);
}

} // extern "C"

namespace soc {
namespace stm32l452xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void adc_interrupt_handler(ADC* a_p_this)
{
    const uint32_t isr = ADC1->ISR;

    if (true == bit_flag::is(isr, ADC_ISR_EOC))
    {
        const bool series_end = bit_flag::is(isr, ADC_ISR_EOS);

        a_p_this->callaback.function(ADC1->DR, series_end, a_p_this, a_p_this->callaback.p_user_data);

        if (true == series_end)
        {
            bit_flag::set(&(ADC1->ISR), ADC_ISR_EOS);
        }
    }
}

bool ADC::enable(Resolution a_resolution,
                 const Asynchronous_clock& a_clock,
                 uint32_t a_irq_priority,
                 uint32_t a_timeout)
{
    cml_assert(nullptr == p_adc_1);
    cml_assert(mcu::Pll_config::Source::unknown != mcu::get_pll_config().source &&
               true == mcu::get_pll_config().pllsai1.r.output_enabled);

    uint32_t start = system_timer::get();

    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE);
    bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_PRESC, static_cast<uint32_t>(a_clock.divider));

    return this->enable(a_resolution, start, a_irq_priority, a_timeout);
}

bool ADC::enable(Resolution a_resolution, const Synchronous_clock& a_clock, uint32_t a_irq_priority, uint32_t a_timeout)
{
    cml_assert(nullptr == p_adc_1);
    cml_assert(Synchronous_clock::Divider::unknown != a_clock.divider);
    cml_assert(Synchronous_clock::Divider::_1 == a_clock.divider ?
                   mcu::Bus_prescalers::AHB::_1 == mcu::get_bus_prescalers().ahb :
                   true);

    uint32_t start = system_timer::get();

    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE, static_cast<uint32_t>(a_clock.divider));

    return this->enable(a_resolution, start, a_irq_priority, a_timeout);
}

void ADC::disable()
{
    ADC1->CR         = 0;
    ADC1_COMMON->CCR = 0;

    bit_flag::set(&(ADC1->CR), ADC_CR_DEEPPWD);
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);

    NVIC_DisableIRQ(ADC1_IRQn);

    p_adc_1 = nullptr;
}

void ADC::set_active_channels(const Channel* a_p_channels, uint32_t a_channels_count)
{
    cml_assert(nullptr != a_p_channels);
    cml_assert(a_channels_count > 0);

    this->clear_active_channels();

    ADC1->SQR1 = a_channels_count - 1;

    for (uint32_t i = 0; i < a_channels_count && i < 4; i++)
    {
        cml_assert(Channel::Id::unknown != a_p_channels[i].id);
        bit_flag::set(&(ADC1->SQR1), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32_t i = 4; i < a_channels_count && i < 9; i++)
    {
        cml_assert(Channel::Id::unknown != a_p_channels[i].id);
        bit_flag::set(&(ADC1->SQR2), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32_t i = 9; i < a_channels_count && i < 14; i++)
    {
        cml_assert(Channel::Id::unknown != a_p_channels[i].id);
        bit_flag::set(&(ADC1->SQR3), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32_t i = 14; i < a_channels_count && i < 16; i++)
    {
        cml_assert(Channel::Id::unknown != a_p_channels[i].id);
        bit_flag::set(&(ADC1->SQR4), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    volatile uint32_t* p_SMPRs = reinterpret_cast<volatile uint32_t*>(&(ADC1->SMPR1));

    for (uint32_t i = 0; i < a_channels_count; i++)
    {
        const uint32_t channel_id        = static_cast<uint32_t>(a_p_channels[i].id);
        const uint32_t sampling_time_val = static_cast<uint32_t>(a_p_channels[i].sampling_time);
        const uint32_t register_index    = channel_id / 10;

        bit_flag::set(&(p_SMPRs[register_index]), sampling_time_val << ((channel_id - (register_index * 10)) * 3));
    }

    bool enable_temperature_sensor = is_channel(Channel::Id::temperature_sensor, a_p_channels, a_channels_count);
    bool enable_voltage_reference  = is_channel(Channel::Id::voltage_reference, a_p_channels, a_channels_count);
    bool enable_battery_voltage    = is_channel(Channel::Id::battery_voltage, a_p_channels, a_channels_count);

    if (true == enable_temperature_sensor)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_TSEN);
        delay::us(120);
    }

    if (true == enable_voltage_reference)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);
    }

    if (true == enable_battery_voltage)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VBATEN);
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

    bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);
}

void ADC::read_polling(uint16_t* a_p_data, uint32_t a_count)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_count > 0);

    cml_assert(this->get_active_channels_count() == a_count);

    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTART);

    for (uint32_t i = 0; i < a_count; i++)
    {
        wait_until::all_bits(&(ADC1->ISR), ADC_ISR_EOC, false);
        a_p_data[i] = static_cast<uint16_t>(ADC1->DR);
    }

    wait_until::all_bits(&(ADC1->ISR), ADC_ISR_EOS, false);
    bit_flag::set(&(ADC1->ISR), ADC_ISR_EOS);

    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(ADC1->CR), ADC_CR_ADSTART);
}

bool ADC::read_polling(uint16_t* a_p_data, uint32_t a_count, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_count > 0);
    cml_assert(a_timeout > 0);

    cml_assert(this->get_active_channels_count() == a_count);

    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTART);

    bool ret       = true;
    uint32_t start = system_timer::get();

    for (uint32_t i = 0; i < a_count && true == ret; i++)
    {
        ret = wait_until::all_bits(&(ADC1->ISR), ADC_ISR_EOC, false, start, a_timeout);

        if (true == ret)
        {
            a_p_data[i] = static_cast<uint16_t>(ADC1->DR);
        }
    }

    if (true == ret)
    {
        ret = wait_until::all_bits(&(ADC1->ISR), ADC_ISR_EOS, false, start, a_timeout - (system_timer::get() - start));

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
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callaback = a_callback;

    bit_flag::set(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTART);
}

void ADC::unregister_conversion_callback()
{
    Interrupt_guard guard;

    bit_flag::set(&(ADC1->CR), ADC_CR_ADSTP);

    bit_flag::clear(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::clear(&(ADC1->CR), ADC_CR_ADSTART);

    this->callaback = { nullptr, nullptr };
}

void ADC::set_resolution(Resolution a_resolution)
{
    bool is_started = bit_flag::is(ADC1->CR, ADC_CR_ADSTART);

    if (true == is_started)
    {
        bit_flag::clear(&(ADC1->CR), ADC_CR_ADSTART);
    }

    bit_flag::set(&(ADC1->CFGR), static_cast<uint32_t>(a_resolution));

    if (true == is_started)
    {
        bit_flag::set(&(ADC1->CR), ADC_CR_ADSTART);
    }
}

bool ADC::enable(Resolution a_resolution, uint32_t a_start, uint32_t a_irq_priority, uint32_t a_timeout)
{
    p_adc_1 = this;

    NVIC_SetPriority(ADC1_IRQn, a_irq_priority);
    NVIC_EnableIRQ(ADC1_IRQn);

    bit_flag::clear(&(ADC1->CR), ADC_CR_DEEPPWD);
    bit_flag::set(&(ADC1->CR), ADC_CR_ADVREGEN);
    delay::us(21u);

    bit_flag::clear(&(ADC1->CR), ADC_CR_ADCALDIF);
    bit_flag::set(&(ADC1->CR), ADC_CR_ADCAL);

    bool ret = wait_until::all_bits(&(ADC1->CR), ADC_CR_ADCAL, true, a_start, a_timeout);

    if (true == ret)
    {
        bit_flag::set(&(ADC1->CFGR), static_cast<uint32_t>(a_resolution));
        bit_flag::set(&(ADC1->CR), ADC_CR_ADEN);

        ret = wait_until::all_bits(
            &(ADC1->ISR), ADC_ISR_ADRDY, false, a_start, a_timeout - (system_timer::get() - a_start));
    }

    if (true == ret)
    {
        bit_flag::set(&(ADC1->ISR), ADC_ISR_ADRDY);
    }

    if (false == ret)
    {
        this->disable();
    }

    return ret;
}

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx