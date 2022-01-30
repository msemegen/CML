/*
 *   Name: ADC.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/ADC/ADC.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/ms_tick_counter.hpp>
#include <cml/utils/wait_until.hpp>

namespace {
using namespace soc::m4::stm32l4;

bool is_channel(ADC::Channel::Id a_type, const ADC::Channel* a_p_channels, std::size_t a_channels_count)
{
    bool found = false;

    for (std::uint32_t i = 0; i < a_channels_count && false == found; i++)
    {
        found = a_type == a_p_channels[i].id;
    }

    return found;
}
} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
#if defined(STM32L412R8) || defined(STM32L412T8) || defined(STM32L412C8) || defined(STM32L412CB) || \
    defined(STM32L412K8) || defined(STM32L412RB) || defined(STM32L412TB) || defined(STM32L422CB) || \
    defined(STM32L422KB) || defined(STM32L422RB) || defined(STM32L422TB)
#define ADC_COMMON_T ADC12_COMMON
#endif
#if defined(STM32L431CB) || defined(STM32L412KB) || defined(STM32L431CC) || defined(STM32L431KB) || \
    defined(STM32L431KC) || defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || \
    defined(STM32L432KB) || defined(STM32L432KC) || defined(STM32L433CB) || defined(STM32L433CC) || \
    defined(STM32L433RB) || defined(STM32L433RC) || defined(STM32L433VC) || defined(STM32L442KC) || \
    defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || defined(STM32L451CC) || \
    defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || defined(STM32L451VC) || \
    defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || defined(STM32L452RC) || \
    defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || defined(STM32L462CE) || \
    defined(STM32L462RE) || defined(STM32L462VE)
#define ADC_COMMON_T ADC1_COMMON
#endif

using namespace cml;
using namespace cml::utils;

void ADC_interrupt_handler(ADC* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(a_p_this));

    const std::uint32_t isr = p_registers->ISR;

    if (true == bit_flag::is(isr, ADC_ISR_EOC))
    {
        const bool series_end = bit_flag::is(isr, ADC_ISR_EOS);

        a_p_this->interrupt.callback.function(p_registers->DR, series_end, a_p_this->interrupt.callback.p_user_data);

        if (true == series_end)
        {
            bit_flag::set(&(p_registers->ISR), ADC_ISR_EOS);
        }
    }
}

bool ADC::enable(Resolution a_resolution, std::uint32_t a_timeout_ms)
{
    cml_assert(true == this->is_created());

    std::uint32_t start = ms_tick_counter::get();

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_DEEPPWD);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADVREGEN);
    delay::us(21u);

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_ADCALDIF);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADCAL);

    bool ret = wait_until::all_bits(
        &(this->p_registers->CR), ADC_CR_ADCAL, true, start, a_timeout_ms - (ms_tick_counter::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->CFGR), ADC_CFGR_RES_Msk, static_cast<std::uint32_t>(a_resolution));
        bit_flag::set(&(this->p_registers->CR), ADC_CR_ADEN);

        ret = wait_until::all_bits(
            &(this->p_registers->ISR), ADC_ISR_ADRDY, false, start, a_timeout_ms - (ms_tick_counter::get() - start));
    }

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->ISR), ADC_ISR_ADRDY);
    }

    return ret;
}

void ADC::disable()
{
    cml_assert(true == this->is_created());

    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits(&(this->p_registers->CR), ADC_CR_ADDIS, true);

    bit_flag::set(&(this->p_registers->CR), ADC_CR_DEEPPWD);
}

void ADC::Polling::disable()
{
    cml_assert(true == this->is_created());

    cml_assert(0x0u == bit_flag::get(static_cast<ADC_TypeDef*>(*(this->p_ADC))->CR, ADC_CR_ADSTART | ADC_CR_JADSTART));

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_ADC));

    p_registers->SQR1 = 0;

    p_registers->SQR2 = 0;
    p_registers->SQR3 = 0;
    p_registers->SQR4 = 0;

    p_registers->SMPR1 = 0;
    p_registers->SMPR2 = 0;

    bit_flag::clear(&(ADC_COMMON_T->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);
    this->enabled = false;
}

void ADC::Polling::enable(const Channel* a_p_channels, std::size_t a_channels_count)
{
    cml_assert(true == this->is_created());

    cml_assert(0x0u == bit_flag::get(static_cast<ADC_TypeDef*>(*(this->p_ADC))->CR, ADC_CR_ADSTART | ADC_CR_JADSTART));

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_ADC));

    for (std::size_t i = 0; i < a_channels_count && i < 4; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(p_registers->SQR1), static_cast<std::uint32_t>(a_p_channels[i].id) << 6 * ((i % 4) + 1));
    }

    for (std::size_t i = 4; i < a_channels_count; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(p_registers->SQR1) + (i + 1) / 5,
                      static_cast<std::uint32_t>(a_p_channels[i].id) << 6 * ((i % 4) + 1));
    }

    for (std::uint32_t i = 0; i < a_channels_count; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Sampling_time>() != a_p_channels[i].sampling_time);

        const std::uint32_t channel_id        = static_cast<std::uint32_t>(a_p_channels[i].id);
        const std::uint32_t sampling_time_val = static_cast<std::uint32_t>(a_p_channels[i].sampling_time);
        const std::uint32_t register_index    = channel_id / 10;

        bit_flag::set(&(p_registers->SMPR1) + register_index,
                      sampling_time_val << ((channel_id - (register_index * 10)) * 3));
    }

    bool enable_temperature_sensor = is_channel(Channel::Id::temperature_sensor, a_p_channels, a_channels_count);
    bool enable_voltage_reference  = is_channel(Channel::Id::voltage_reference, a_p_channels, a_channels_count);
    bool enable_battery_voltage    = is_channel(Channel::Id::battery_voltage, a_p_channels, a_channels_count);

    if (true == enable_temperature_sensor)
    {
        bit_flag::set(&(ADC_COMMON_T->CCR), ADC_CCR_TSEN);
        delay::us(120);
    }

    if (true == enable_voltage_reference)
    {
        bit_flag::set(&(ADC_COMMON_T->CCR), ADC_CCR_VREFEN);
    }

    if (true == enable_battery_voltage)
    {
        bit_flag::set(&(ADC_COMMON_T->CCR), ADC_CCR_VBATEN);
    }

    this->enabled = true;
}

void ADC::Polling::read(Mode a_mode, uint16_t* a_p_buffer, std::size_t a_buffer_capacity)
{
    cml_assert(true == this->is_created());

    cml_assert(nullptr != a_p_buffer);
    cml_assert(a_buffer_capacity > 0);

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_ADC));

    bit_flag::set(&(p_registers->CFGR), ADC_CFGR_CONT, static_cast<uint32_t>(a_mode));
    bit_flag::set(&(p_registers->CR), ADC_CR_ADSTART);

    std::uint32_t i = 0;
    while (i < a_buffer_capacity)
    {
        if (true == bit_flag::is(p_registers->ISR, ADC_ISR_EOC))
        {
            a_p_buffer[i++] = static_cast<std::uint16_t>(p_registers->DR);
        }
    }

    wait_until::all_bits(&(p_registers->ISR), ADC_ISR_EOS, false);
    bit_flag::set(&(p_registers->ISR), ADC_ISR_EOS);

    bit_flag::set(&(p_registers->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(p_registers->CR), ADC_CR_ADSTART);
}

bool ADC::Polling::read(Mode a_mode, uint16_t* a_p_buffer, std::size_t a_buffer_capacity, uint32_t a_timeout)
{
    cml_assert(true == this->is_created());

    cml_assert(nullptr != a_p_buffer);
    cml_assert(a_buffer_capacity > 0);
    cml_assert(a_timeout > 0);

    std::uint32_t start = ms_tick_counter::get();

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_ADC));

    std::size_t i = 0;

    bit_flag::set(&(p_registers->CFGR), ADC_CFGR_CONT, static_cast<uint32_t>(a_mode));
    bit_flag::set(&(p_registers->CR), ADC_CR_ADSTART);

    while (ms_tick_counter::get() < start + a_timeout && i < a_buffer_capacity)
    {
        if (true == bit_flag::is(p_registers->ISR, ADC_ISR_EOC))
        {
            a_p_buffer[i++] = static_cast<std::uint16_t>(p_registers->DR);
        }
    }

    bool ret = wait_until::all_bits(
        &(p_registers->ISR), ADC_ISR_EOS, false, start, a_timeout - (ms_tick_counter::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(p_registers->ISR), ADC_ISR_EOS);
    }

    bit_flag::set(&(p_registers->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(p_registers->CR), ADC_CR_ADSTART);

    return ret && i == a_buffer_capacity;
}

void ADC::Interrupt::disable()
{
    cml_assert(true == this->is_created());

    cml_assert(0x0u == bit_flag::get(static_cast<ADC_TypeDef*>(*(this->p_ADC))->CR, ADC_CR_ADSTART | ADC_CR_JADSTART));

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_ADC));

    NVIC_DisableIRQ(this->irqn);

    p_registers->SQR1 = 0;

    p_registers->SQR2 = 0;
    p_registers->SQR3 = 0;
    p_registers->SQR4 = 0;

    p_registers->SMPR1 = 0;
    p_registers->SMPR2 = 0;

    bit_flag::clear(&(ADC_COMMON_T->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);

    this->clear_irq_context();
}

void ADC::Interrupt::register_callack(Mode a_mode, const Callback& a_callback)
{
    cml_assert(true == this->is_created());

    cml_assert(nullptr != a_callback.function);

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_ADC));

    Interrupt_guard guard;

    this->callback = a_callback;

    bit_flag::set(&(p_registers->CFGR), ADC_CFGR_CONT, static_cast<uint32_t>(a_mode));
    bit_flag::set(&(p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(p_registers->CR), ADC_CR_ADSTART);
}

void ADC::Interrupt::unregister_callback()
{
    cml_assert(true == this->is_created());

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_ADC));

    Interrupt_guard guard;

    bit_flag::set(&(p_registers->CR), ADC_CR_ADSTP);

    bit_flag::clear(&(p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::clear(&(p_registers->CR), ADC_CR_ADSTART);

    this->callback = { nullptr, nullptr };
}

void ADC::Interrupt::enable(const IRQ_config& a_irq_config, const Channel* a_p_channels, std::size_t a_channels_count)
{
    cml_assert(true == this->is_created());

    cml_assert(0x0u == bit_flag::get(static_cast<ADC_TypeDef*>(*(this->p_ADC))->CR, ADC_CR_ADSTART | ADC_CR_JADSTART));

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_ADC));

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
        bit_flag::set(&(ADC_COMMON_T->CCR), ADC_CCR_TSEN);
        delay::us(120);
    }

    if (true == enable_voltage_reference)
    {
        bit_flag::set(&(ADC_COMMON_T->CCR), ADC_CCR_VREFEN);
    }

    if (true == enable_battery_voltage)
    {
        bit_flag::set(&(ADC_COMMON_T->CCR), ADC_CCR_VBATEN);
    }

    this->set_irq_context();

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif