/*
 *   Name: Polling.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/ADC/Polling.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace {

using namespace soc::m4::stm32l4;

bool is_channel(Polling<ADC>::Channel::Id a_type,
                const Polling<ADC>::Channel* a_p_channels,
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

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;
using namespace cml::utils;

void Polling<ADC>::disable()
{
    cml_assert(false == bit_flag::is(static_cast<ADC_TypeDef*>(*(this->p_adc))->CR, ADC_CR_ADSTART) &&
               false == bit_flag::is(static_cast<ADC_TypeDef*>(*(this->p_adc))->CR, ADC_CR_JADSTART));

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_adc));

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
}

void Polling<ADC>::enable(const Channel* a_p_channels, std::size_t a_channels_count)
{
    cml_assert(false == bit_flag::is(static_cast<ADC_TypeDef*>(*(this->p_adc))->CR, ADC_CR_ADSTART) &&
               false == bit_flag::is(static_cast<ADC_TypeDef*>(*(this->p_adc))->CR, ADC_CR_JADSTART));

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_adc));

    p_registers->SQR1 = a_channels_count - 1;

    volatile uint32_t* p_SQRs  = &(p_registers->SQR1);
    volatile uint32_t* p_SMPRs = &(p_registers->SMPR1);

    for (std::size_t i = 0; i < a_channels_count && i < 16u; i++)
    {
        cml_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(p_SQRs[i / 4]), static_cast<std::uint32_t>(a_p_channels[i].id) << 6 * ((i % 4) + 1));
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
}

void Polling<ADC>::read(Mode a_mode, uint16_t* a_p_buffer, std::size_t a_buffer_capacity)
{
    cml_assert(nullptr != a_p_buffer);
    cml_assert(a_buffer_capacity > 0);

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_adc));

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

bool Polling<ADC>::read(Mode a_mode, uint16_t* a_p_buffer, std::size_t a_buffer_capacity, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_buffer);
    cml_assert(a_buffer_capacity > 0);
    cml_assert(a_timeout > 0);

    std::uint32_t start = system_timer::get();

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(this->p_adc));

    std::size_t i = 0;

    bit_flag::set(&(p_registers->CFGR), ADC_CFGR_CONT, static_cast<uint32_t>(a_mode));
    bit_flag::set(&(p_registers->CR), ADC_CR_ADSTART);

    while (system_timer::get() < start + a_timeout && i < a_buffer_capacity)
    {
        if (true == bit_flag::is(p_registers->ISR, ADC_ISR_EOC))
        {
            a_p_buffer[i++] = static_cast<std::uint16_t>(p_registers->DR);
        }
    }

    bool ret =
        wait_until::all_bits(&(p_registers->ISR), ADC_ISR_EOS, false, start, a_timeout - (system_timer::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(p_registers->ISR), ADC_ISR_EOS);
    }

    bit_flag::set(&(p_registers->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(p_registers->CR), ADC_CR_ADSTART);

    return ret && i == a_buffer_capacity;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif