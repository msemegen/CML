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
#include <soc/m4/Interrupt_guard.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/tick_counter.hpp>
#include <cml/utils/wait_until.hpp>

namespace {
using namespace cml;
using namespace cml::utils;
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

void polling_read(ADC_TypeDef* a_p_registers,
                  ADC::Mode a_mode,
                  std::uint16_t* a_p_buffer,
                  std::size_t a_buffer_capacity,
                  std::size_t a_group_size)
{
    cml_assert(nullptr != a_p_buffer);
    cml_assert(a_buffer_capacity > 0);
    cml_assert((ADC::Mode::discontinuous == a_mode && a_group_size > 0 && a_group_size <= 8) ||
               ADC::Mode::discontinuous != a_mode);

    bit_flag::set(
        &(a_p_registers->CFGR), ADC_CFGR_CONT | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_Msk, static_cast<uint32_t>(a_mode));

    if (ADC::Mode::discontinuous == a_mode)
    {
        bit_flag::set(&(a_p_registers->CFGR), (a_group_size - 1) << ADC_CFGR_DISCNUM_Pos);
    }

    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTART);

    std::uint32_t i = 0;
    while (i < a_buffer_capacity)
    {
        if (true == bit_flag::is(a_p_registers->ISR, ADC_ISR_EOC))
        {
            a_p_buffer[i++] = static_cast<std::uint16_t>(a_p_registers->DR);
        }
    }

    wait_until::all_bits(&(a_p_registers->ISR), ADC_ISR_EOS, false);
    bit_flag::set(&(a_p_registers->ISR), ADC_ISR_EOS);

    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(a_p_registers->CR), ADC_CR_ADSTART);
}

bool polling_read(ADC_TypeDef* a_p_registers,
                  ADC::Mode a_mode,
                  std::uint16_t* a_p_buffer,
                  std::size_t a_buffer_capacity,
                  std::size_t a_group_size,
                  Milliseconds a_timeout)
{
    cml_assert(nullptr != a_p_buffer);
    cml_assert(a_buffer_capacity > 0);
    cml_assert((ADC::Mode::discontinuous == a_mode && a_group_size > 0 && a_group_size <= 8) ||
               ADC::Mode::discontinuous != a_mode);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    std::size_t i = 0;

    bit_flag::set(
        &(a_p_registers->CFGR), ADC_CFGR_CONT | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_Msk, static_cast<uint32_t>(a_mode));

    if (ADC::Mode::discontinuous == a_mode)
    {
        bit_flag::set(&(a_p_registers->CFGR), (a_group_size - 1) << ADC_CFGR_DISCNUM_Pos);
    }

    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTART);

    while (tick_counter::get() < start + a_timeout && i < a_buffer_capacity)
    {
        if (true == bit_flag::is(a_p_registers->ISR, ADC_ISR_EOC))
        {
            a_p_buffer[i++] = static_cast<std::uint16_t>(a_p_registers->DR);
        }
    }

    bool ret = wait_until::all_bits(
        &(a_p_registers->ISR), ADC_ISR_EOS, false, start, a_timeout - (tick_counter::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(a_p_registers->ISR), ADC_ISR_EOS);
    }

    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(a_p_registers->CR), ADC_CR_ADSTART);

    return ret && i == a_buffer_capacity;
}

} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
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

        a_p_this->read_callback.function(p_registers->DR, series_end, a_p_this->read_callback.p_user_data);

        if (true == series_end)
        {
            bit_flag::set(&(p_registers->ISR), ADC_ISR_EOS);
        }
    }
}

bool ADC::enable(Resolution a_resolution,
                 const Channel* a_p_channels,
                 std::size_t a_channels_count,
                 Milliseconds a_timeout)
{
    cml_assert(true == this->is_created());
    cml_assert(0x0u == bit_flag::get(this->p_registers->CR, ADC_CR_ADSTART | ADC_CR_JADSTART));

    Milliseconds start = tick_counter::get();

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_DEEPPWD);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADVREGEN);
    tick_counter::delay(21_us);

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_ADCALDIF);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADCAL);

    bool ret = wait_until::all_bits(
        &(this->p_registers->CR), ADC_CR_ADCAL, true, start, a_timeout - (tick_counter::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->CFGR), ADC_CFGR_RES_Msk, static_cast<std::uint32_t>(a_resolution));
        bit_flag::set(&(this->p_registers->CR), ADC_CR_ADEN);

        ret = wait_until::all_bits(
            &(this->p_registers->ISR), ADC_ISR_ADRDY, false, start, a_timeout - (tick_counter::get() - start));
    }

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->ISR), ADC_ISR_ADRDY);

        this->p_registers->SQR1 = a_channels_count - 1;

        volatile uint32_t* p_SQRs  = &(this->p_registers->SQR1);
        volatile uint32_t* p_SMPRs = &(this->p_registers->SMPR1);

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
            tick_counter::delay(120_us);
        }

        if (true == enable_voltage_reference)
        {
            bit_flag::set(&(ADC_COMMON_T->CCR), ADC_CCR_VREFEN);
        }

        if (true == enable_battery_voltage)
        {
            bit_flag::set(&(ADC_COMMON_T->CCR), ADC_CCR_VBATEN);
        }
    }

    return ret;
}

void ADC::disable()
{
    cml_assert(true == this->is_created());
    cml_assert(0x0u == bit_flag::get(this->p_registers->CR, ADC_CR_ADSTART | ADC_CR_JADSTART));

    this->p_registers->SQR1 = 0;

    this->p_registers->SQR2 = 0;
    this->p_registers->SQR3 = 0;
    this->p_registers->SQR4 = 0;

    this->p_registers->SMPR1 = 0;
    this->p_registers->SMPR2 = 0;

    bit_flag::clear(&(ADC_COMMON_T->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);

    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits(&(this->p_registers->CR), ADC_CR_ADDIS, true);

    bit_flag::set(&(this->p_registers->CR), ADC_CR_DEEPPWD);
}

template<> void ADC::Polling::read<ADC::Mode::single>(uint16_t* a_p_buffer, std::size_t a_buffer_capacity)
{
    polling_read(this->p_ADC->p_registers, ADC::Mode::single, a_p_buffer, a_buffer_capacity, 0u);
}
template<> void ADC::Polling::read<ADC::Mode::continuous>(uint16_t* a_p_buffer, std::size_t a_buffer_capacity)
{
    polling_read(this->p_ADC->p_registers, ADC::Mode::continuous, a_p_buffer, a_buffer_capacity, 0u);
}
template<> void ADC::Polling::read<ADC::Mode::discontinuous>(uint16_t* a_p_buffer,
                                                             std::size_t a_buffer_capacity,
                                                             std::size_t a_group_size)
{
    polling_read(this->p_ADC->p_registers, ADC::Mode::discontinuous, a_p_buffer, a_buffer_capacity, a_group_size);
}

template<> bool
ADC::Polling::read<ADC::Mode::single>(uint16_t* a_p_buffer, std::size_t a_buffer_capacity, cml::Milliseconds a_timeout)
{
    return polling_read(this->p_ADC->p_registers, ADC::Mode::single, a_p_buffer, a_buffer_capacity, 0u, a_timeout);
}
template<> bool ADC::Polling::read<ADC::Mode::continuous>(uint16_t* a_p_buffer,
                                                          std::size_t a_buffer_capacity,
                                                          cml::Milliseconds a_timeout)
{
    return polling_read(this->p_ADC->p_registers, ADC::Mode::continuous, a_p_buffer, a_buffer_capacity, 0u, a_timeout);
}
template<> bool ADC::Polling::read<ADC::Mode::discontinuous>(uint16_t* a_p_buffer,
                                                             std::size_t a_buffer_capacity,
                                                             std::size_t a_group_size,
                                                             cml::Milliseconds a_timeout)
{
    return polling_read(
        this->p_ADC->p_registers, ADC::Mode::discontinuous, a_p_buffer, a_buffer_capacity, a_group_size, a_timeout);
}

void ADC::Interrupt::enable(const IRQ_config& a_irq_config)
{
    this->set_irq_context();

    NVIC_SetPriority(
        this->p_ADC->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_ADC->irqn);
}

void ADC::Interrupt::disable()
{
    this->read_stop();
    this->clear_irq_context();

    NVIC_DisableIRQ(this->p_ADC->irqn);
}

template<> void ADC::Interrupt::read_start<ADC::Mode::single>(const Read_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->p_ADC->read_callback = a_callback;

    bit_flag::clear(&(this->p_ADC->p_registers->CFGR), ADC_CFGR_CONT);
    bit_flag::set(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);
}
template<> void ADC::Interrupt::read_start<ADC::Mode::continuous>(const Read_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->p_ADC->read_callback = a_callback;

    bit_flag::set(&(this->p_ADC->p_registers->CFGR), ADC_CFGR_CONT);
    bit_flag::set(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);
}
template<>
void ADC::Interrupt::read_start<ADC::Mode::discontinuous>(const Read_callback& a_callback, std::size_t a_group_size)
{
    cml_assert(nullptr != a_callback.function);
    cml_assert(a_group_size > 0u);

    Interrupt_guard guard;

    this->p_ADC->read_callback = a_callback;

    bit_flag::set(
        &(this->p_ADC->p_registers->CFGR), ADC_CFGR_CONT | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_Msk, ADC_CFGR_DISCEN);
    bit_flag::set(&(this->p_ADC->p_registers->CFGR), (a_group_size - 1) << ADC_CFGR_DISCNUM_Pos);

    bit_flag::set(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);
}

void ADC::Interrupt::read_stop()
{
    Interrupt_guard guard;

    bit_flag::clear(&(this->p_ADC->p_registers->CFGR), ADC_CFGR_CONT | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_Msk);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTP);

    bit_flag::clear(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::clear(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);

    this->p_ADC->read_callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif