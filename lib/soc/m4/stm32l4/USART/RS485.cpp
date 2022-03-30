/*
 *   Name: RS485.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/USART/RS485.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/debug/assertion.hpp>
#include <cml/utils/tick_counter.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

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

        if (Event_flag_t::overrun == (a_enabled_events & Event_flag_t::overrun))
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
    if (true == cml::bit_flag::is(isr, USART_ISR_CMF))
    {
        bit_flag::set(&(a_p_registers->ICR), USART_ICR_CMCF);

        if (Event_flag_t::address_matched == (a_enabled_events & Event_flag_t::address_matched))
        {
            ret |= Event_flag_t::address_matched;
        }
    }

    return ret;
}

constexpr RS485::Event_flag all_polling_result_event_flags =
    RS485::Event_flag::framing_error | RS485::Event_flag::idle | RS485::Event_flag::noise_detected |
    RS485::Event_flag::overrun | RS485::Event_flag::parity_error | RS485::Event_flag::transfer_complete |
    RS485::Event_flag::address_matched;
} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;
using namespace cml::utils;

void RS485_interrupt_handler(RS485* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(a_p_this));

    const std::uint32_t isr = p_registers->ISR;
    const std::uint32_t cr1 = p_registers->CR1;

    if (nullptr != a_p_this->event_callback.function)
    {
        RS485::Event_flag event =
            get_Event_flag_and_clear<RS485::Event_flag>(p_registers, a_p_this->enabled_interrupt_events);

        if (RS485::Event_flag::none != event)
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

bool RS485::enable(const Enable_config& a_config, Milliseconds a_timeout)
{
    cml_assert(true == this->is_created());

    cml_assert(0 != a_config.baud_rate);
    cml_assert(0 != a_config.clock_freq_Hz);

    cml_assert(various::get_enum_incorrect_value<Enable_config::Stop_bits>() != a_config.stop_bits);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Oversampling>() != a_config.oversampling);

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

    this->p_registers->CR3 = USART_CR3_ONEBIT;
    this->p_registers->CR2 =
        static_cast<std::uint32_t>(a_config.stop_bits) | (a_config.address << USART_CR2_ADD_Pos) | USART_CR2_ADDM7;

    this->p_registers->CR1 = static_cast<std::uint32_t>(a_config.oversampling) | USART_CR1_M0 | USART_CR1_UE |
                             USART_CR1_TE | USART_CR1_RE | USART_CR1_MME | USART_CR1_WAKE;

    this->p_registers->RQR = USART_RQR_MMRQ;

    bool ret = wait_until::all_bits(&(this->p_registers->ISR),
                                    USART_ISR_TEACK | USART_ISR_REACK | USART_ISR_RWU | USART_ISR_IDLE,
                                    false,
                                    start,
                                    a_timeout - (tick_counter::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->ICR), USART_ICR_IDLECF);
    }

    return ret;
}

void RS485::disable()
{
    cml_assert(true == this->is_created());

    this->p_registers->CR1 = 0;
    this->p_registers->CR2 = 0;
    this->p_registers->CR3 = 0;
}

RS485::Polling::Result RS485::Polling::transmit(std::uint8_t a_address,
                                                const void* a_p_data,
                                                std::size_t a_data_size_in_words,
                                                GPIO::Out::Pin* a_p_flow_control_pin)
{
    cml_assert(a_address <= 0x7F);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_TCCF);

    std::size_t words = 0;
    bool error        = false;
    Event_flag event  = Event_flag::none;

    a_p_flow_control_pin->set_level(GPIO::Level::high);

    while (false == bit_flag::is(this->p_RS485->p_registers->ISR, USART_ISR_TC) && false == error)
    {
        if (true == bit_flag::is(this->p_RS485->p_registers->ISR, USART_ISR_TXE))
        {
            if (words == 0)
            {
                this->p_RS485->p_registers->TDR = (static_cast<uint16_t>(a_address) | static_cast<uint16_t>(0x100u));
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                this->p_RS485->p_registers->TDR = (static_cast<const uint8_t*>(a_p_data)[words - 1]);
                words++;
            }
        }

        error = is_error(this->p_RS485->p_registers);
    }

    a_p_flow_control_pin->set_level(GPIO::Level::low);

    if (true == error)
    {
        event = get_Event_flag_and_clear<Event_flag>(this->p_RS485->p_registers, all_polling_result_event_flags);
    }

    return { event, words };
}

RS485::Polling::Result RS485::Polling::transmit(std::uint8_t a_address,
                                                const void* a_p_data,
                                                std::size_t a_data_size_in_words,
                                                GPIO::Out::Pin* a_p_flow_control_pin,
                                                Milliseconds a_timeout)
{
    cml_assert(a_address <= 0x7F);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_TCCF);

    std::size_t words = 0;
    bool error        = false;
    Event_flag event  = Event_flag::none;

    a_p_flow_control_pin->set_level(GPIO::Level::high);

    while (false == bit_flag::is(this->p_RS485->p_registers->ISR, USART_ISR_TC) && false == error &&
           a_timeout > tick_counter::get() - start)
    {
        if (true == bit_flag::is(this->p_RS485->p_registers->ISR, USART_ISR_TXE))
        {
            if (words == 0)
            {
                this->p_RS485->p_registers->TDR = (static_cast<uint16_t>(a_address) | static_cast<uint16_t>(0x100u));
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                this->p_RS485->p_registers->TDR = (static_cast<const uint8_t*>(a_p_data)[words - 1]);
                words++;
            }
        }

        error = is_error(this->p_RS485->p_registers);
    }

    a_p_flow_control_pin->set_level(GPIO::Level::low);

    if (true == error)
    {
        event = get_Event_flag_and_clear<Event_flag>(this->p_RS485->p_registers, all_polling_result_event_flags);
    }

    return { event, words };
}

RS485::Polling::Result
RS485::Polling::receive(void* a_p_data, std::size_t a_data_size_in_words, GPIO::Out::Pin* a_p_flow_control_pin)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words = 0;
    bool error        = false;
    Event_flag event  = Event_flag::none;

    a_p_flow_control_pin->set_level(GPIO::Level::low);

    while (false == bit_flag::is(this->p_RS485->p_registers->ISR, USART_ISR_IDLE) && false == error)
    {
        if (true == bit_flag::is(this->p_RS485->p_registers->ISR, USART_ISR_RXNE))
        {
            if (0 == words)
            {
                bit_flag::set(&(this->p_RS485->p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                static_cast<uint8_t*>(a_p_data)[words - 1] = (this->p_RS485->p_registers->RDR & 0xFFu);
                words++;
            }
            else
            {
                bit_flag::set(&(this->p_RS485->p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_error(this->p_RS485->p_registers);
    }

    bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_CMCF);

    if (true == error)
    {
        event = get_Event_flag_and_clear<Event_flag>(this->p_RS485->p_registers, all_polling_result_event_flags);
    }

    return { event, words };
}

RS485::Polling::Result RS485::Polling::receive(void* a_p_data,
                                               std::size_t a_data_size_in_words,
                                               GPIO::Out::Pin* a_p_flow_control_pin,
                                               Milliseconds a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words = 0;
    bool error        = false;
    Event_flag event  = Event_flag::none;

    a_p_flow_control_pin->set_level(GPIO::Level::low);

    while (false == bit_flag::is(this->p_RS485->p_registers->ISR, USART_ISR_IDLE) && false == error &&
           a_timeout > tick_counter::get() - start)
    {
        if (true == bit_flag::is(this->p_RS485->p_registers->ISR, USART_ISR_RXNE))
        {
            if (0 == words)
            {
                bit_flag::set(&(this->p_RS485->p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                static_cast<uint8_t*>(a_p_data)[words - 1] = (this->p_RS485->p_registers->RDR & 0xFFu);
                words++;
            }
            else
            {
                bit_flag::set(&(this->p_RS485->p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_error(this->p_RS485->p_registers);
    }

    bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_CMCF);

    if (true == error)
    {
        event = get_Event_flag_and_clear<Event_flag>(this->p_RS485->p_registers, all_polling_result_event_flags);
    }

    return { event, words };
}

void RS485::Interrupt::enable(const IRQ_config& a_irq_config)
{
    this->set_irq_context();

    NVIC_SetPriority(
        this->p_RS485->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_RS485->irqn);
}

void RS485::Interrupt::disable()
{
    NVIC_DisableIRQ(this->p_RS485->irqn);

    this->clear_irq_context();
}

void RS485::Interrupt::transmit_start(const Transmit_callback& a_callback, GPIO::Out::Pin* a_p_flow_control_pin)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->p_RS485->transmit_callback = a_callback;

    a_p_flow_control_pin->set_level(GPIO::Level::high);

    bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->ICR), USART_ICR_TCCF);
    bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);
}

void RS485::Interrupt::receive_start(const Receive_callback& a_callback, GPIO::Out::Pin* a_p_flow_control_pin)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->p_RS485->receive_callback = a_callback;

    a_p_flow_control_pin->set_level(GPIO::Level::low);

    bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->ICR), USART_ICR_IDLECF);
    bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
}

void RS485::Interrupt::register_Event_callback(const Event_callback& a_callback, Event_flag a_enabled_events)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->p_RS485->event_callback           = a_callback;
    this->p_RS485->enabled_interrupt_events = a_enabled_events;

    if (Event_flag::parity_error == (a_enabled_events | Event_flag::parity_error))
    {
        bit_flag::set(&(this->p_RS485->p_registers->CR1), USART_CR1_PEIE);
        bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_PECF);
    }

    if (Event_flag::idle == (a_enabled_events | Event_flag::idle))
    {
        bit_flag::set(&(this->p_RS485->p_registers->CR1), USART_CR1_IDLEIE);
        bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_IDLECF);
    }

    if (Event_flag::address_matched == (a_enabled_events | Event_flag::address_matched))
    {
        bit_flag::set(&(this->p_RS485->p_registers->CR1), USART_CR1_CMIE);
        bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_CMCF);
    }

    if (Event_flag::transfer_complete == (a_enabled_events | Event_flag::transfer_complete))
    {
        bit_flag::set(&(this->p_RS485->p_registers->CR1), USART_CR1_TCIE);
        bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_TCCF);
    }

    if (Event_flag::framing_error == (a_enabled_events | Event_flag::framing_error) ||
        Event_flag::noise_detected == (a_enabled_events | Event_flag::noise_detected) ||
        Event_flag::overrun == (a_enabled_events | Event_flag::overrun))
    {
        bit_flag::set(&(this->p_RS485->p_registers->CR3), USART_CR3_EIE);
        bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF);
    }
}

void RS485::Interrupt::transmit_stop()
{
    Interrupt_guard guard;

    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);

    this->p_RS485->transmit_callback = { nullptr, nullptr };
}

void RS485::Interrupt::receive_stop()
{
    Interrupt_guard guard;

    bit_flag::set(&(this->p_RS485->p_registers->ICR), USART_ICR_CMCF);
    bit_flag::clear(&(this->p_RS485->p_registers->CR1), USART_CR1_RXNEIE);

    this->p_RS485->receive_callback = { nullptr, nullptr };
}

void RS485::Interrupt::unregister_Event_callback()
{
    Interrupt_guard guard;

    if (Event_flag::parity_error == (this->p_RS485->enabled_interrupt_events | Event_flag::parity_error))
    {
        bit_flag::clear(&(this->p_RS485->p_registers->CR1), USART_CR1_PEIE);
    }

    if (Event_flag::idle == (this->p_RS485->enabled_interrupt_events | Event_flag::idle))
    {
        bit_flag::clear(&(this->p_RS485->p_registers->CR1), USART_CR1_IDLEIE);
    }

    if (Event_flag::framing_error == (this->p_RS485->enabled_interrupt_events | Event_flag::framing_error) ||
        Event_flag::noise_detected == (this->p_RS485->enabled_interrupt_events | Event_flag::noise_detected) ||
        Event_flag::overrun == (this->p_RS485->enabled_interrupt_events | Event_flag::overrun))
    {
        bit_flag::clear(&(this->p_RS485->p_registers->CR3), USART_CR3_EIE);
    }

    this->p_RS485->event_callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif