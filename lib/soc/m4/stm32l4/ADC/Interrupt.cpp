/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/ADC/Interrupt.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include "..\RNG\Interrupt.hpp"

#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>
#include "..\I2C\Interrupt.hpp"
#include "..\SPI\Interrupt.hpp"

namespace {

using namespace soc::m4::stm32l4;

Interrupt<ADC>* adcs[] = { nullptr,
#if defined(STM32L412xx) || defined(STM32L422xx)
                           nullptr
#endif
};

bool is_channel(Interrupt<ADC>::Channel::Id a_type,
                const Interrupt<ADC>::Channel* a_p_channels,
                std::size_t a_channels_count)
{
    bool found = false;

    for (std::uint32_t i = 0; i < a_channels_count && false == found; i++)
    {
        found = a_type == a_p_channels[i].id;
    }

    return found;
}

} // namespace

extern "C" {

#if defined(STM32L412xx) || defined(STM32L422xx)
void ADC1_2_IRQHandler()
{
    cml_assert(nullptr != adcs[0] || nullptr != adcs[1]);

    if (nullptr != adcs[0])
    {
        adc_interrupt_handler(adcs[0]);
    }

    if (nullptr != adcs[1])
    {
        adc_interrupt_handler(adcs[1]);
    }
}
#else
void ADC1_IRQHandler()
{
    cml_assert(nullptr != adcs[0]);
    adc_interrupt_handler(adcs[0]);
}
#endif
}

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;
using namespace cml::utils;

void adc_interrupt_handler(Interrupt<ADC>* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(a_p_this->p_adc));

    const std::uint32_t isr = p_registers->ISR;

    if (true == bit_flag::is(isr, ADC_ISR_EOC))
    {
        const bool series_end = bit_flag::is(isr, ADC_ISR_EOS);

        a_p_this->conversion_callback.function(
            p_registers->DR, series_end, a_p_this->p_adc, a_p_this->conversion_callback.p_user_data);

        if (true == series_end)
        {
            bit_flag::set(&(p_registers->ISR), ADC_ISR_EOS);
        }
    }
}

void Interrupt<ADC>::disable()
{
    cml_assert(false == bit_flag::is(static_cast<ADC_TypeDef*>(*(this->p_adc))->CR, ADC_CR_ADSTART) &&
               false == bit_flag::is(static_cast<ADC_TypeDef*>(*(this->p_adc))->CR, ADC_CR_JADSTART));

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_adc));

    NVIC_DisableIRQ(this->irqn);

    p_registers->SQR1 = 0;

    p_registers->SQR2 = 0;
    p_registers->SQR3 = 0;
    p_registers->SQR4 = 0;

    p_registers->SMPR1 = 0;
    p_registers->SMPR2 = 0;
#if defined(STM32L412xx) || defined(STM32L422xx)
    bit_flag::clear(&(ADC12_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);
#else
    bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);
#endif

    adcs[p_adc->get_idx()] = nullptr;
}

void Interrupt<ADC>::register_callack(Mode a_mode, const Conversion_callback& a_callback)
{
    cml_assert((Mode::none != a_mode && nullptr != a_callback.function) ||
               (Mode::none == a_mode && nullptr == a_callback.function));

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_adc));

    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->conversion_callback = a_callback;

        bit_flag::set(&(p_registers->CFGR), ADC_CFGR_CONT, static_cast<uint32_t>(a_mode));
        bit_flag::set(&(p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
        bit_flag::set(&(p_registers->CR), ADC_CR_ADSTART);
    }
    else
    {
        bit_flag::set(&(p_registers->CR), ADC_CR_ADSTP);

        bit_flag::clear(&(p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
        bit_flag::clear(&(p_registers->CR), ADC_CR_ADSTART);

        this->conversion_callback = a_callback;
    }
}

void Interrupt<ADC>::enable(const IRQ_config& a_irq_config, const Channel* a_p_channels, std::size_t a_channels_count)
{
    cml_assert(false == bit_flag::is(static_cast<ADC_TypeDef*>(*(this->p_adc))->CR, ADC_CR_ADSTART) &&
               false == bit_flag::is(static_cast<ADC_TypeDef*>(*(this->p_adc))->CR, ADC_CR_JADSTART));

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_adc));

    p_registers->SQR1 = a_channels_count - 1;

    volatile uint32_t* p_SQRs  = &(p_registers->SQR1);
    volatile uint32_t* p_SMPRs = &(p_registers->SMPR1);

    for (std::size_t i = 0; i < a_channels_count && i < 4; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(p_SQRs[0]), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (std::size_t i = 4; i < a_channels_count; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(p_SQRs[(i + 1) / 5]), static_cast<uint32_t>(a_p_channels[i].id) << 6 * ((i + 1) % 5));
    }

    for (std::uint32_t i = 0; i < a_channels_count; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Sampling_time>() != a_p_channels[i].sampling_time);

        const std::uint32_t channel_id        = static_cast<std::uint32_t>(a_p_channels[i].id);
        const std::uint32_t sampling_time_val = static_cast<std::uint32_t>(a_p_channels[i].sampling_time);
        const std::uint32_t register_index    = channel_id / 10;

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

    adcs[this->p_adc->get_idx()] = this;

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif