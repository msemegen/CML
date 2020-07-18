/*
    Name: ADC.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <soc/stm32l452xx/peripherals/ADC.hpp>

//soc
#include <soc/counter.hpp>

//cml
#include <cml/debug/assert.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/wait.hpp>

#ifdef CML_ASSERT
#include <soc/stm32l452xx/mcu.hpp>
#endif // CML_ASSERT

namespace
{

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

} // namespace ::

extern "C"
{

void ADC1_IRQHandler()
{
    assert(nullptr != p_adc_1);
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

    if (true == is_flag(isr, ADC_ISR_EOC))
    {
        const bool series_end = is_flag(isr, ADC_ISR_EOS);
        const bool ret = a_p_this->callaback.function(ADC1->DR, series_end, a_p_this->callaback.p_user_data);

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

bool ADC::enable(Resolution a_resolution,
                 const Asynchronous_clock& a_clock,
                 uint32_t a_irq_priority,
                 time::tick a_timeout)
{
    assert(nullptr == p_adc_1);
    assert(mcu::Pll_config::Source::unknown != mcu::get_pll_config().source &&
           true == mcu::get_pll_config().pllsai1.r.output_enabled);

    time::tick start = counter::get();

    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    clear_flag(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE);
    set_flag(&(ADC1_COMMON->CCR), ADC_CCR_PRESC, static_cast<uint32_t>(a_clock.divider));

    return this->enable(a_resolution, start, a_irq_priority, a_timeout);
}

bool ADC::enable(Resolution a_resolution,
                 const Synchronous_clock& a_clock,
                 uint32_t a_irq_priority,
                 time::tick a_timeout)
{
    assert(nullptr == p_adc_1);
    assert(Synchronous_clock::Divider::unknown != a_clock.divider);
    assert(Synchronous_clock::Divider::_1 == a_clock.divider ?
           mcu::Bus_prescalers::AHB::_1 == mcu::get_bus_prescalers().ahb :
           true);

    time::tick start = counter::get();

    set_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    set_flag(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE, static_cast<uint32_t>(a_clock.divider));

    return this->enable(a_resolution, start, a_irq_priority, a_timeout);
}

void ADC::disable()
{
    ADC1->CR         = 0;
    ADC1_COMMON->CCR = 0;

    set_flag(&(ADC1->CR), ADC_CR_DEEPPWD);
    clear_flag(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);

    NVIC_DisableIRQ(ADC1_IRQn);

    p_adc_1 = nullptr;
}

void ADC::set_active_channels(const Channel* a_p_channels, uint32_t a_channels_count)
{
    assert(nullptr != p_adc_1);
    assert(nullptr != a_p_channels);
    assert(a_channels_count > 0);

    this->clear_active_channels();

    ADC1->SQR1 = a_channels_count - 1;

    for (uint32_t i = 0; i < a_channels_count && i < 4; i++)
    {
        assert(Channel::Id::unknown != a_p_channels[i].id);
        set_flag(&(ADC1->SQR1), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32_t i = 4; i < a_channels_count && i < 9; i++)
    {
        assert(Channel::Id::unknown != a_p_channels[i].id);
        set_flag(&(ADC1->SQR2), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32_t i = 9; i < a_channels_count && i < 14; i++)
    {
        assert(Channel::Id::unknown != a_p_channels[i].id);
        set_flag(&(ADC1->SQR3), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32_t i = 14; i < a_channels_count && i < 16; i++)
    {
        assert(Channel::Id::unknown != a_p_channels[i].id);
        set_flag(&(ADC1->SQR4), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    volatile uint32_t* p_SMPRs = reinterpret_cast<volatile uint32_t*>(&(ADC1->SMPR1));

    for (uint32_t i = 0; i < a_channels_count; i++)
    {
        const uint32_t channel_id        = static_cast<uint32_t>(a_p_channels[i].id);
        const uint32_t sampling_time_val = static_cast<uint32_t>(a_p_channels[i].sampling_time);
        const uint32_t register_index    = channel_id / 10;

        set_flag(&(p_SMPRs[register_index]), sampling_time_val << ((channel_id - (register_index * 10)) * 3));
    }

    bool enable_temperature_sensor = is_channel(Channel::Id::temperature_sensor, a_p_channels, a_channels_count);
    bool enable_voltage_reference  = is_channel(Channel::Id::voltage_reference, a_p_channels, a_channels_count);
    bool enable_battery_voltage    = is_channel(Channel::Id::battery_voltage, a_p_channels, a_channels_count);

    if (true == enable_temperature_sensor)
    {
        set_flag(&(ADC1_COMMON->CCR), ADC_CCR_TSEN);
        delay::us(120);
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
    assert(nullptr != p_adc_1);

    ADC1->SQR1 = 0;

    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->SQR4 = 0;

    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;

    clear_flag(&(ADC1_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);
}

void ADC::read_polling(uint16_t* a_p_data, uint32_t a_count)
{
    assert(nullptr != p_adc_1);
    assert(nullptr != a_p_data);
    assert(a_count > 0);

    assert(this->get_active_channels_count() == a_count);

    set_flag(&(ADC1->CR), ADC_CR_ADSTART);

    for (uint32_t i = 0; i < a_count; i++)
    {
        wait::until(&(ADC1->ISR), ADC_ISR_EOC, false);
        a_p_data[i] = static_cast<uint16_t>(ADC1->DR);
    }

    wait::until(&(ADC1->ISR), ADC_ISR_EOS, false);
    set_flag(&(ADC1->ISR), ADC_ISR_EOS);

    set_flag(&(ADC1->CR), ADC_CR_ADSTP);
    clear_flag(&(ADC1->CR), ADC_CR_ADSTART);
}

bool ADC::read_polling(uint16_t* a_p_data, uint32_t a_count, time::tick a_timeout)
{
    assert(nullptr != p_adc_1);
    assert(nullptr != a_p_data);
    assert(a_count > 0);
    assert(a_timeout > 0);

    assert(this->get_active_channels_count() == a_count);

    set_flag(&(ADC1->CR), ADC_CR_ADSTART);

    bool ret         = true;
    time::tick start = counter::get();

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

    this->callaback = a_callback;

    set_flag(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    set_flag(&(ADC1->CR), ADC_CR_ADSTART);
}

void ADC::stop_read_it()
{
    assert(nullptr != p_adc_1);

    clear_flag(&(ADC1->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    clear_flag(&(ADC1->CR), ADC_CR_ADSTART);

    this->callaback = { nullptr, nullptr };
}

void ADC::set_resolution(Resolution a_resolution)
{
    assert(nullptr != p_adc_1);

    bool is_started = is_flag(ADC1->CR, ADC_CR_ADSTART);

    if (true == is_started)
    {
        clear_flag(&(ADC1->CR), ADC_CR_ADSTART);
    }

    set_flag(&(ADC1->CFGR), static_cast<uint32_t>(a_resolution));

    if (true == is_started)
    {
        set_flag(&(ADC1->CR), ADC_CR_ADSTART);
    }
}

bool ADC::enable(Resolution a_resolution, time::tick a_start, uint32_t a_irq_priority, time::tick a_timeout)
{
    p_adc_1 = this;

    NVIC_SetPriority(ADC1_IRQn, a_irq_priority);
    NVIC_EnableIRQ(ADC1_IRQn);

    clear_flag(&(ADC1->CR), ADC_CR_DEEPPWD);
    set_flag(&(ADC1->CR), ADC_CR_ADVREGEN);
    delay::us(21u);

    clear_flag(&(ADC1->CR), ADC_CR_ADCALDIF);
    set_flag(&(ADC1->CR), ADC_CR_ADCAL);

    bool ret = wait::until(&(ADC1->CR), ADC_CR_ADCAL, true, a_start, a_timeout);

    if (true == ret)
    {
        set_flag(&(ADC1->CFGR), static_cast<uint32_t>(a_resolution));
        set_flag(&(ADC1->CR), ADC_CR_ADEN);

        ret = wait::until(&(ADC1->ISR), ADC_ISR_ADRDY, false, a_start, a_timeout);
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

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx