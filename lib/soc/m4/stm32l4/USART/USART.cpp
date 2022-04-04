/*
 *   Name: USART.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/USART/USART.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/tick_counter.hpp>
#include <cml/utils/wait_until.hpp>

namespace {
using namespace cml;
using namespace soc::m4::stm32l4;

bool is_error(USART_TypeDef* a_p_registers)
{
    return bit::is_any(a_p_registers->ISR, USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
}

template<typename Event_flag_t>
Event_flag_t get_Event_flag_and_clear(USART_TypeDef* a_p_registers, Event_flag_t a_enabled_events)
{
    Event_flag_t ret        = Event_flag_t::none;
    const std::uint32_t isr = a_p_registers->ISR;

    if (true == bit_flag::is(isr, USART_ISR_PE))
    {
        bit_flag::set(&(a_p_registers->ICR), USART_ICR_PECF);

        if (Event_flag_t::parity_error == (a_enabled_events & Event_flag_t::parity_error))
        {
            ret |= Event_flag_t::parity_error;
        }
    }
    if (true == bit_flag::is(isr, USART_ISR_FE))
    {
        bit_flag::set(&(a_p_registers->ICR), USART_ICR_FECF);

        if (Event_flag_t::framing_error == (a_enabled_events & Event_flag_t::framing_error))
        {
            ret |= Event_flag_t::framing_error;
        }
    }
    if (true == bit_flag::is(isr, USART_ISR_ORE))
    {
        bit_flag::set(&(a_p_registers->ICR), USART_ICR_ORECF);

        if (Event_flag_t::overrun == (a_enabled_events | Event_flag_t::overrun))
        {
            ret |= Event_flag_t::overrun;
        }
    }
    if (true == bit_flag::is(isr, USART_ISR_NE))
    {
        bit_flag::set(&(a_p_registers->ICR), USART_ICR_NECF);

        if (Event_flag_t::noise_detected == (a_enabled_events & Event_flag_t::noise_detected))
        {
            ret |= Event_flag_t::noise_detected;
        }
    }
    if (true == bit_flag::is(isr, USART_ISR_IDLE))
    {
        bit_flag::set(&(a_p_registers->ICR), USART_ICR_IDLECF);

        if (Event_flag_t::idle == (a_enabled_events & Event_flag_t::idle))
        {
            ret |= Event_flag_t::idle;
        }
    }
    if (true == bit_flag::is(isr, USART_ISR_TC))
    {
        bit_flag::set(&(a_p_registers->ICR), USART_ICR_TCCF);

        if (Event_flag_t::transfer_complete == (a_enabled_events & Event_flag_t::transfer_complete))
        {
            ret |= Event_flag_t::transfer_complete;
        }
    }

    return ret;
}

constexpr USART::Event_flag all_polling_result_event_flags =
    USART::Event_flag::framing_error | USART::Event_flag::idle | USART::Event_flag::noise_detected |
    USART::Event_flag::overrun | USART::Event_flag::parity_error | USART::Event_flag::transfer_complete;
} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;
using namespace cml::utils;

void USART_interrupt_handler(USART* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*a_p_this);

    const std::uint32_t isr = p_registers->ISR;
    const std::uint32_t cr1 = p_registers->CR1;

    if (nullptr != a_p_this->event_callback.function)
    {
        USART::Event_flag event =
            get_Event_flag_and_clear<USART::Event_flag>(p_registers, a_p_this->enabled_interrupt_events);

        if (USART::Event_flag::none != event)
        {
            a_p_this->event_callback.function(event, a_p_this->event_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_TXE) && true == bit_flag::is(cr1, USART_CR1_TXEIE))
        {
            a_p_this->transmit_callback.function(&(p_registers->TDR), a_p_this->transmit_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_RXNE) && true == bit_flag::is(cr1, USART_CR1_RXNEIE))
        {
            a_p_this->receive_callback.function(p_registers->RDR, a_p_this->receive_callback.p_user_data);
        }
    }
}

bool USART::enable(const Enable_config& a_config, const Frame_config& a_frame_format, Milliseconds a_timeout)
{
    cml_assert(true == this->is_created());

    cml_assert(0 != a_config.baud_rate);
    cml_assert(0 != a_config.clock_freq_Hz);

    cml_assert(various::get_enum_incorrect_value<Enable_config::Flow_control_flag>() != a_config.flow_control);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Stop_bits>() != a_config.stop_bits);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Sampling_method>() != a_config.sampling_method);

    cml_assert(various::get_enum_incorrect_value<Frame_config::Parity>() != a_frame_format.parity);
    cml_assert(various::get_enum_incorrect_value<Frame_config::Word_length>() != a_frame_format.word_length);

    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    switch (a_config.oversampling)
    {
        case Enable_config::Oversampling::_16: {
            this->p_registers->BRR = a_config.clock_freq_Hz / a_config.baud_rate;
        }
        break;

        case Enable_config::Oversampling::_8: {
            const std::uint32_t usartdiv = 2 * a_config.clock_freq_Hz / a_config.baud_rate;
            this->p_registers->BRR       = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;
    }

    this->p_registers->CR2 = static_cast<std::uint32_t>(a_config.stop_bits);
    this->p_registers->CR3 =
        static_cast<std::uint32_t>(a_config.flow_control) | static_cast<std::uint32_t>(a_config.sampling_method);

    this->p_registers->CR1 = static_cast<std::uint32_t>(a_config.oversampling) |
                             static_cast<std::uint32_t>(a_config.mode) |
                             static_cast<std::uint32_t>(a_frame_format.parity) |
                             static_cast<std::uint32_t>(a_frame_format.word_length) | USART_CR1_UE;

    std::uint32_t wait_flag =
        (true == bit_flag::is(this->p_registers->CR1, USART_CR1_RE) ? (USART_ISR_REACK | USART_ISR_IDLE) : 0) |
        (true == bit_flag::is(this->p_registers->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0);

    bool ret = wait_until::all_bits(
        &(this->p_registers->ISR), wait_flag, false, start, a_timeout - (tick_counter::get() - start));

    if (true == ret && bit_flag::is(this->p_registers->ISR, USART_ISR_IDLE))
    {
        bit_flag::set(&(this->p_registers->ICR), USART_ICR_IDLECF);
    }

    return ret;
}

void USART::disable()
{
    cml_assert(true == this->is_created());

    if (true == this->interrupt.is_enabled())
    {
        this->disable();
    }

    this->p_registers->CR1 = 0;
    this->p_registers->CR2 = 0;
    this->p_registers->CR3 = 0;
}

USART::Polling::Result USART::Polling::transmit(const void* a_p_data, std::size_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_USART));

    bit_flag::set(&(p_registers->ICR), USART_ICR_TCCF);

    std::size_t words = 0;
    bool error        = false;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_TC) && false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_TXE) && words < a_data_size_in_words)
        {
            if (false == bit::is(p_registers->CR1, USART_CR1_PCE_Pos) &&
                (true == bit::is(p_registers->CR1, USART_CR1_M0_Pos) &&
                 false == bit::is(p_registers->CR1, USART_CR1_M1_Pos)))
            {
                p_registers->TDR = (static_cast<const std::uint16_t*>(a_p_data)[words++]) & 0x1FFu;
            }
            else
            {
                p_registers->TDR = (static_cast<const std::uint8_t*>(a_p_data)[words++]) & 0xFFu;
            }
        }

        error = is_error(p_registers);
    }

    return { get_Event_flag_and_clear<USART::Event_flag>(p_registers, all_polling_result_event_flags), words };
}

USART::Polling::Result
USART::Polling::transmit(const void* a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_USART));

    bit_flag::set(&(p_registers->ICR), USART_ICR_TCCF);

    std::size_t words = 0;
    bool error        = false;
    Event_flag event  = Event_flag::none;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_TC) && false == error &&
           a_timeout > tick_counter::get() - start)
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_TXE) && words < a_data_size_in_words)
        {
            if (false == bit::is(p_registers->CR1, USART_CR1_PCE_Pos) &&
                (true == bit::is(p_registers->CR1, USART_CR1_M0_Pos) &&
                 false == bit::is(p_registers->CR1, USART_CR1_M1_Pos)))
            {
                p_registers->TDR = (static_cast<const std::uint16_t*>(a_p_data)[words++]) & 0x1FFu;
            }
            else
            {
                p_registers->TDR = (static_cast<const std::uint8_t*>(a_p_data)[words++]) & 0xFFu;
            }
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        event = get_Event_flag_and_clear<USART::Event_flag>(p_registers, all_polling_result_event_flags);
    }

    return { event, words };
}

USART::Polling::Result USART::Polling::receive(void* a_p_data, std::size_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_USART));

    bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words     = 0;
    bool error            = false;
    Event_flag bus_status = Event_flag::none;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_IDLE) && false == error)
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_RXNE))
        {
            if (words < a_data_size_in_words)
            {
                if (false == bit::is(p_registers->CR1, USART_CR1_PCE_Pos) &&
                    (true == bit::is(p_registers->CR1, USART_CR1_M0_Pos) &&
                     false == bit::is(p_registers->CR1, USART_CR1_M1_Pos)))
                {
                    static_cast<std::uint16_t*>(a_p_data)[words++] = (p_registers->RDR & 0x1FFu);
                }
                else
                {
                    static_cast<std::uint8_t*>(a_p_data)[words++] = (p_registers->RDR & 0xFFu);
                }
            }
            else
            {
                bit_flag::set(&(p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Event_flag_and_clear<USART::Event_flag>(p_registers, all_polling_result_event_flags);
    }

    return { bus_status, words };
}

USART::Polling::Result USART::Polling::receive(void* a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_USART));

    bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words     = 0;
    bool error            = false;
    Event_flag bus_status = Event_flag::none;

    while (false == bit_flag::is(p_registers->ISR, USART_ISR_IDLE) && false == error &&
           a_timeout > tick_counter::get() - start)
    {
        if (true == bit_flag::is(p_registers->ISR, USART_ISR_RXNE))
        {
            if (words < a_data_size_in_words)
            {
                if (false == bit::is(p_registers->CR1, USART_CR1_PCE_Pos) &&
                    (true == bit::is(p_registers->CR1, USART_CR1_M0_Pos) &&
                     false == bit::is(p_registers->CR1, USART_CR1_M1_Pos)))
                {
                    static_cast<std::uint16_t*>(a_p_data)[words++] = (p_registers->RDR & 0x1FFu);
                }
                else
                {
                    static_cast<std::uint8_t*>(a_p_data)[words++] = (p_registers->RDR & 0xFFu);
                }
            }
            else
            {
                bit_flag::set(&(p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_error(p_registers);
    }

    if (true == error)
    {
        bus_status = get_Event_flag_and_clear<USART::Event_flag>(p_registers, all_polling_result_event_flags);
    }

    return { bus_status, words };
}

void USART::Interrupt::enable(const IRQ_config& a_irq_config)
{
    cml_assert(various::get_enum_incorrect_value<IRQ_config::Mode>() != a_irq_config.mode);

    this->set_irq_context();

    if (IRQ_config::Mode::enabled == a_irq_config.mode)
    {
        NVIC_SetPriority(
            this->p_USART->irqn,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
        NVIC_EnableIRQ(this->p_USART->irqn);
    }
}

void USART::Interrupt::disable()
{
    this->transmit_stop();
    this->receive_stop();

    this->clear_irq_context();

    NVIC_DisableIRQ(this->p_USART->irqn);
}

void USART::Interrupt::transmit_start(const Transmit_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);
    cml_assert(true == bit_flag::is(this->p_USART->p_registers->CR1, USART_CR1_TE));

    Interrupt_guard guard;

    this->p_USART->transmit_callback = a_callback;

    bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_USART))->ICR), USART_ICR_TCCF);
    bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_USART))->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);
}

void USART::Interrupt::receive_start(const Receive_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);
    cml_assert(true == bit_flag::is(this->p_USART->p_registers->CR1, USART_CR1_RE));

    Interrupt_guard guard;

    this->p_USART->receive_callback = a_callback;

    bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_USART))->CR1), USART_CR1_RXNEIE);
}

void USART::Interrupt::event_listening_start(const Event_callback& a_callback, Event_flag a_enabled_events)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->p_USART->event_callback           = a_callback;
    this->p_USART->enabled_interrupt_events = a_enabled_events;

    if (Event_flag::parity_error == (a_enabled_events | Event_flag::parity_error))
    {
        bit_flag::set(&(this->p_USART->p_registers->CR1), USART_CR1_PEIE);
        bit_flag::set(&(this->p_USART->p_registers->ICR), USART_ICR_PECF);
    }

    if (Event_flag::idle == (a_enabled_events | Event_flag::idle))
    {
        bit_flag::set(&(this->p_USART->p_registers->CR1), USART_CR1_IDLEIE);
        bit_flag::set(&(this->p_USART->p_registers->ICR), USART_ICR_IDLECF);
    }

    if (Event_flag::transfer_complete == (a_enabled_events | Event_flag::transfer_complete))
    {
        bit_flag::set(&(this->p_USART->p_registers->CR1), USART_CR1_TCIE);
        bit_flag::set(&(this->p_USART->p_registers->ICR), USART_ICR_TCCF);
    }

    if (Event_flag::framing_error == (a_enabled_events | Event_flag::framing_error) ||
        Event_flag::noise_detected == (a_enabled_events | Event_flag::noise_detected) ||
        Event_flag::overrun == (a_enabled_events | Event_flag::overrun))
    {
        bit_flag::set(&(this->p_USART->p_registers->CR3), USART_CR3_EIE);
        bit_flag::set(&(this->p_USART->p_registers->ICR), USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF);
    }
}

void USART::Interrupt::transmit_stop()
{
    Interrupt_guard guard;

    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_USART))->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);

    this->p_USART->transmit_callback = { nullptr, nullptr };
}

void USART::Interrupt::receive_stop()
{
    Interrupt_guard guard;

    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_USART))->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);

    this->p_USART->receive_callback = { nullptr, nullptr };
}

void USART::Interrupt::event_listening_stop()
{
    Interrupt_guard guard;

    if (Event_flag::parity_error == (this->p_USART->enabled_interrupt_events | Event_flag::parity_error))
    {
        bit_flag::clear(&(this->p_USART->p_registers->CR1), USART_CR1_PEIE);
    }

    if (Event_flag::idle == (this->p_USART->enabled_interrupt_events | Event_flag::idle))
    {
        bit_flag::clear(&(this->p_USART->p_registers->CR1), USART_CR1_IDLEIE);
    }

    if (Event_flag::framing_error == (this->p_USART->enabled_interrupt_events | Event_flag::framing_error) ||
        Event_flag::noise_detected == (this->p_USART->enabled_interrupt_events | Event_flag::noise_detected) ||
        Event_flag::overrun == (this->p_USART->enabled_interrupt_events | Event_flag::overrun))
    {
        bit_flag::clear(&(this->p_USART->p_registers->CR3), USART_CR3_EIE);
    }

    this->p_USART->enabled_interrupt_events = Event_flag::none;
    this->p_USART->event_callback           = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif