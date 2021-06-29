/*
 *   Name: ADC.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/peripherals/ADC.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/wait_until.hpp>

#ifdef CML_ASSERT_ENABLED
#include <soc/m4/mcu.hpp>
#endif // CML_ASSERT_ENABLED

namespace {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

using namespace cml;
using namespace soc::m4::stm32l4::peripherals;

struct Controller
{
    ADC_TypeDef* p_registers = nullptr;
    ADC* p_adc               = nullptr;
};

bool is_channel(ADC::Channel::Id a_type, const ADC::Channel* a_p_channels, uint32_t a_channels_count)
{
    bool found = false;

    for (uint32_t i = 0; i < a_channels_count && false == found; i++)
    {
        found = a_type == a_p_channels[i].id;
    }

    return found;
}

Controller controllers[] = { { ADC1, nullptr },
#if defined(STM32L412xx) || defined(STM32L422xx)
                             { ADC2, nullptr }
#endif
};

ADC_TypeDef* get_adc_ptr(ADC::Id a_id)
{
    return controllers[static_cast<uint32_t>(a_id)].p_registers;
}

#endif

} // namespace

extern "C" {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

#if defined(STM32L412xx) || defined(STM32L422xx)
void ADC1_2_IRQHandler()
{
    cml_assert(nullptr != controllers[0].p_adc || nullptr != controllers[1].p_adc);

    if (nullptr != controllers[0].p_adc)
    {
        adc_interrupt_handler(controllers[0].p_adc);
    }

    if (nullptr != controllers[1].p_adc)
    {
        adc_interrupt_handler(controllers[1].p_adc);
    }
}
#else
void ADC1_IRQHandler()
{
    cml_assert(nullptr != controllers[0].p_adc);
    adc_interrupt_handler(controllers[0].p_adc);
}
#endif

#endif

} // extern "C"

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

using namespace cml;
using namespace cml::utils;

void adc_interrupt_handler(ADC* a_p_this)
{
    const uint32_t isr = get_adc_ptr(a_p_this->get_id())->ISR;

    if (true == bit_flag::is(isr, ADC_ISR_EOC))
    {
        const bool series_end = bit_flag::is(isr, ADC_ISR_EOS);

        a_p_this->callaback.function(
            get_adc_ptr(a_p_this->get_id())->DR, series_end, a_p_this, a_p_this->callaback.p_user_data);

        if (true == series_end)
        {
            bit_flag::set(&(get_adc_ptr(a_p_this->get_id())->ISR), ADC_ISR_EOS);
        }
    }
}

bool ADC::enable(Resolution a_resolution, uint32_t a_irq_priority, uint32_t a_timeout)
{
    uint32_t start = system_timer::get();

#if defined(STM32L412xx) || defined(STM32L422xx)
    NVIC_SetPriority(ADC1_2_IRQn, a_irq_priority);
    NVIC_EnableIRQ(ADC1_2_IRQn);
#else
    NVIC_SetPriority(ADC1_IRQn, a_irq_priority);
    NVIC_EnableIRQ(ADC1_IRQn);
#endif

    bit_flag::clear(&(get_adc_ptr(this->id)->CR), ADC_CR_DEEPPWD);
    bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADVREGEN);
    delay::us(21u);

    bit_flag::clear(&(get_adc_ptr(this->id)->CR), ADC_CR_ADCALDIF);
    bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADCAL);

    bool ret = wait_until::all_bits(&(get_adc_ptr(this->id)->CR), ADC_CR_ADCAL, true, start, a_timeout);

    if (true == ret)
    {
        bit_flag::set(&(get_adc_ptr(this->id)->CFGR), static_cast<uint32_t>(a_resolution));
        bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADEN);

        ret = wait_until::all_bits(
            &(get_adc_ptr(this->id)->ISR), ADC_ISR_ADRDY, false, start, a_timeout - (system_timer::get() - start));
    }

    if (true == ret)
    {
        bit_flag::set(&(get_adc_ptr(this->id)->ISR), ADC_ISR_ADRDY);
        controllers[static_cast<uint32_t>(this->id)].p_adc = this;
    }

    if (false == ret)
    {
        this->disable();
    }

    return ret;
}

void ADC::disable()
{
    get_adc_ptr(this->id)->CR = 0;
    bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_DEEPPWD);

#if defined(STM32L412xx) || defined(STM32L422xx)
    NVIC_DisableIRQ(ADC1_2_IRQn);
#else
    NVIC_DisableIRQ(ADC1_IRQn);
#endif

    controllers[static_cast<uint32_t>(this->id)].p_adc = nullptr;
}

void ADC::set_active_channels(const Channel* a_p_channels, uint32_t a_channels_count)
{
    cml_assert(nullptr != a_p_channels);
    cml_assert(a_channels_count > 0);

    this->clear_active_channels();

    get_adc_ptr(this->id)->SQR1 = a_channels_count - 1;

    for (uint32_t i = 0; i < a_channels_count && i < 4; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(get_adc_ptr(this->id)->SQR1), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32_t i = 4; i < a_channels_count && i < 9; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(get_adc_ptr(this->id)->SQR2), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32_t i = 9; i < a_channels_count && i < 14; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(get_adc_ptr(this->id)->SQR3), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (uint32_t i = 14; i < a_channels_count && i < 16; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(get_adc_ptr(this->id)->SQR4), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    volatile uint32_t* p_SMPRs = reinterpret_cast<volatile uint32_t*>(&(get_adc_ptr(this->id)->SMPR1));

    for (uint32_t i = 0; i < a_channels_count; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Sampling_time>() != a_p_channels[i].sampling_time);

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
#if defined(STM32L412xx) || defined(STM32L422xx)
        bit_flag::set(&(ADC12_COMMON->CCR), ADC_CCR_TSEN);
#else
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_TSEN);
#endif
        delay::us(120);
    }

    if (true == enable_voltage_reference)
    {
#if defined(STM32L412xx) || defined(STM32L422xx)
        bit_flag::set(&(ADC12_COMMON->CCR), ADC_CCR_VREFEN);
#else
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);
#endif
    }

    if (true == enable_battery_voltage)
    {
#if defined(STM32L412xx) || defined(STM32L422xx)
        bit_flag::set(&(ADC12_COMMON->CCR), ADC_CCR_VBATEN);
#else
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VBATEN);
#endif
    }
}

void ADC::clear_active_channels()
{
    get_adc_ptr(this->id)->SQR1 = 0;

    get_adc_ptr(this->id)->SQR2 = 0;
    get_adc_ptr(this->id)->SQR3 = 0;
    get_adc_ptr(this->id)->SQR4 = 0;

    get_adc_ptr(this->id)->SMPR1 = 0;
    get_adc_ptr(this->id)->SMPR2 = 0;
#if defined(STM32L412xx) || defined(STM32L422xx)
    bit_flag::clear(&(ADC12_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);
#else
    bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);
#endif
}

void ADC::read_polling(uint16_t* a_p_data, uint32_t a_count)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_count > 0);

    cml_assert(this->get_active_channels_count() == a_count);

    bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTART);

    for (uint32_t i = 0; i < a_count; i++)
    {
        wait_until::all_bits(&(get_adc_ptr(this->id)->ISR), ADC_ISR_EOC, false);
        a_p_data[i] = static_cast<uint16_t>(get_adc_ptr(this->id)->DR);
    }

    wait_until::all_bits(&(get_adc_ptr(this->id)->ISR), ADC_ISR_EOS, false);
    bit_flag::set(&(get_adc_ptr(this->id)->ISR), ADC_ISR_EOS);

    bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTART);
}

bool ADC::read_polling(uint16_t* a_p_data, uint32_t a_count, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_count > 0);
    cml_assert(a_timeout > 0);

    cml_assert(this->get_active_channels_count() == a_count);

    bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTART);

    bool ret       = true;
    uint32_t start = system_timer::get();

    for (uint32_t i = 0; i < a_count && true == ret; i++)
    {
        ret = wait_until::all_bits(&(get_adc_ptr(this->id)->ISR), ADC_ISR_EOC, false, start, a_timeout);

        if (true == ret)
        {
            a_p_data[i] = static_cast<uint16_t>(get_adc_ptr(this->id)->DR);
        }
    }

    if (true == ret)
    {
        ret = wait_until::all_bits(
            &(get_adc_ptr(this->id)->ISR), ADC_ISR_EOS, false, start, a_timeout - (system_timer::get() - start));

        if (true == ret)
        {
            bit_flag::set(&(get_adc_ptr(this->id)->ISR), ADC_ISR_EOS);
        }
    }

    bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTART);

    return ret;
}

void ADC::register_conversion_callback(const Conversion_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callaback = a_callback;

    bit_flag::set(&(get_adc_ptr(this->id)->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTART);
}

void ADC::unregister_conversion_callback()
{
    Interrupt_guard guard;

    bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTP);

    bit_flag::clear(&(get_adc_ptr(this->id)->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::clear(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTART);

    this->callaback = { nullptr, nullptr };
}

uint32_t ADC::get_active_channels_count() const
{
    return (get_adc_ptr(this->id)->SQR1 & 0xFu) + 1;
}

void ADC::set_resolution(Resolution a_resolution)
{
    bool is_started = bit_flag::is(get_adc_ptr(this->id)->CR, ADC_CR_ADSTART);

    if (true == is_started)
    {
        bit_flag::clear(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTART);
    }

    bit_flag::set(&(get_adc_ptr(this->id)->CFGR), static_cast<uint32_t>(a_resolution));

    if (true == is_started)
    {
        bit_flag::set(&(get_adc_ptr(this->id)->CR), ADC_CR_ADSTART);
    }
}
#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace soc::m4::stm32l4::peripherals;

void rcc<ADC>::enable(Clock_source a_source, Prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);

    switch (a_source)
    {
        case Clock_source::pclk: {
            cml_assert(a_prescaler <= Prescaler::_4);
#if defined(STM32L412xx) || defined(STM32L422xx)
            bit_flag::set(&(ADC12_COMMON->CCR), ADC_CCR_CKMODE_Msk, static_cast<uint32_t>(a_prescaler) << 16ul);
#else
            bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE_Msk, static_cast<uint32_t>(a_prescaler) << 16ul);
#endif
        }
        break;
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Clock_source::pllsai: {
            bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE_Msk);
            bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_PRESC_Msk, static_cast<uint32_t>(a_prescaler) << 18ul);
        }
        break;
#endif
    }

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_ADCSMEN);
    }
}

void rcc<ADC>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_ADCSMEN);
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4