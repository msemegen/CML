/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/USART/Interrupt.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/USART/common.hpp>

// cml
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void usart_interrupt_handler(Interrupt<USART>* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(a_p_this->get_handle()));

    const std::uint32_t isr = p_registers->ISR;
    const std::uint32_t cr1 = p_registers->CR1;
    const std::uint32_t cr3 = p_registers->CR3;

    if (nullptr != a_p_this->tx.callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_TXE) && true == bit_flag::is(cr1, USART_CR1_TXEIE))
        {
            a_p_this->tx.callback.function(&(p_registers->TDR), false, a_p_this->tx.callback.p_user_data);
        }

        if (true == bit_flag::is(isr, USART_ISR_TC) && true == bit_flag::is(cr1, USART_CR1_TCIE))
        {
            a_p_this->tx.callback.function(nullptr, true, a_p_this->tx.callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->rx.callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_RXNE) && true == bit_flag::is(cr1, USART_CR1_RXNEIE))
        {
            a_p_this->rx.callback.function(p_registers->RDR, false, a_p_this->rx.callback.p_user_data);
        }
        else if (true == bit_flag::is(isr, USART_ISR_IDLE) && true == bit_flag::is(cr1, USART_CR1_IDLEIE))
        {
            bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);
            a_p_this->rx.callback.function(0x0u, true, a_p_this->rx.callback.p_user_data);
        }
    }

    if (true == bit_flag::is(cr3, USART_CR3_EIE) && true == bit_flag::is(cr1, USART_CR1_PEIE))
    {
        Interrupt<USART>::Status::Callback::Flag status =
            get_Bus_status_flag<Interrupt<USART>::Status::Callback::Flag>(p_registers);

        if (status != Interrupt<USART>::Status::Callback::Flag::ok)
        {
            if (nullptr != a_p_this->status.callback.function)
            {
                a_p_this->status.callback.function(status, a_p_this->status.callback.p_user_data);
            }

            bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
        }
    }
}

void Interrupt<USART>::enable(const IRQ_config& a_irq_config)
{
    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    this->set_irq_context();
}

void Interrupt<USART>::disable()
{
    NVIC_DisableIRQ(this->irqn);

    this->clear_irq_context();
}

void Interrupt<USART>::TX::register_callback(const Interrupt<USART>::TX::Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()));
    bit_flag::set(&(p_registers->ICR), USART_ICR_TCCF);
    bit_flag::set(&(p_registers->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);
}

void Interrupt<USART>::TX::unregister_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()))->CR1),
                    USART_CR1_TXEIE | USART_CR1_TCIE);

    this->callback = { nullptr, nullptr };
}

void Interrupt<USART>::RX::register_callback(const Interrupt<USART>::RX::Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()));
    bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);
    bit_flag::set(&(p_registers->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
}

void Interrupt<USART>::RX::unregister_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()))->CR1),
                    USART_CR1_RXNEIE | USART_CR1_IDLEIE);

    this->callback = { nullptr, nullptr };
}

void Interrupt<USART>::Status::register_callback(const Interrupt<USART>::Status::Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()));
    bit_flag::set(&(p_registers->CR1), USART_CR1_PEIE);
    bit_flag::set(&(p_registers->CR3), USART_CR3_EIE);
}

void Interrupt<USART>::Status::unregister_callback()
{
    Interrupt_guard guard;

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()));
    bit_flag::clear(&(p_registers->CR1), USART_CR1_PEIE);
    bit_flag::clear(&(p_registers->CR3), USART_CR3_EIE);

    this->callback = { nullptr, nullptr };
}

void rs485_interrupt_handler(Interrupt<RS485>* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(a_p_this->get_handle()));

    const std::uint32_t isr = p_registers->ISR;
    const std::uint32_t cr1 = p_registers->CR1;
    const std::uint32_t cr3 = p_registers->CR3;

    if (nullptr != a_p_this->tx.callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_TXE) && true == bit_flag::is(cr1, USART_CR1_TXEIE))
        {
            a_p_this->tx.callback.function(&(p_registers->TDR), false, a_p_this->tx.callback.p_user_data);
        }

        if (true == bit_flag::is(isr, USART_ISR_TC) && true == bit_flag::is(cr1, USART_CR1_TCIE))
        {
            a_p_this->tx.callback.function(nullptr, true, a_p_this->tx.callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->rx.callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_RXNE) && true == bit_flag::is(cr1, USART_CR1_RXNEIE))
        {
            volatile uint16_t rdr = p_registers->RDR;

            if (false == bit_flag::is(rdr, 0x100))
            {
                a_p_this->rx.callback.function(rdr, false, a_p_this->rx.callback.p_user_data);
            }
        }
        else if (true == bit_flag::is(isr, USART_ISR_IDLE) && true == bit_flag::is(cr1, USART_CR1_IDLEIE))
        {
            bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);
            a_p_this->rx.callback.function(0x0u, true, a_p_this->rx.callback.p_user_data);
        }
    }

    if (true == bit_flag::is(cr3, USART_CR3_EIE) && true == bit_flag::is(cr1, USART_CR1_PEIE))
    {
        Interrupt<RS485>::Status::Callback::Flag status =
            get_Bus_status_flag<Interrupt<RS485>::Status::Callback::Flag>(p_registers);

        if (status != Interrupt<RS485>::Status::Callback::Flag::ok)
        {
            if (nullptr != a_p_this->status.callback.function)
            {
                a_p_this->status.callback.function(status, a_p_this->status.callback.p_user_data);
            }

            bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
        }
    }
}

void Interrupt<RS485>::enable(const IRQ_config& a_irq_config)
{
    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    this->set_irq_context();
}

void Interrupt<RS485>::disable()
{
    NVIC_DisableIRQ(this->irqn);

    this->clear_irq_context();
}

void Interrupt<RS485>::TX::register_callback(const Interrupt<RS485>::TX::Callback& a_callback,
                                             GPIO::Out::Pin* a_p_flow_control_pin)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()));
    bit_flag::set(&(p_registers->ICR), USART_ICR_TCCF);
    bit_flag::set(&(p_registers->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);

    a_p_flow_control_pin->set_level(GPIO::Level::high);
}

void Interrupt<RS485>::TX::unregister_callback(GPIO::Out::Pin* a_p_flow_control_pin)
{
    Interrupt_guard guard;

    a_p_flow_control_pin->set_level(GPIO::Level::low);
    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()))->CR1),
                    USART_CR1_TCIE | USART_CR1_TXEIE);

    this->callback = { nullptr, nullptr };
}

void Interrupt<RS485>::RX::register_callback(const Interrupt<RS485>::RX::Callback& a_callback,
                                             GPIO::Out::Pin* a_p_flow_control_pin)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()));
    bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);
    bit_flag::set(&(p_registers->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);

    a_p_flow_control_pin->set_level(GPIO::Level::low);
}

void Interrupt<RS485>::RX::unregister_callback(GPIO::Out::Pin* a_p_flow_control_pin)
{
    Interrupt_guard guard;

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()));
    bit_flag::set(&(p_registers->ICR), USART_ICR_CMCF);
    bit_flag::clear(&(p_registers->CR1), USART_CR1_RXNEIE);

    a_p_flow_control_pin->set_level(GPIO::Level::high);

    this->callback = { nullptr, nullptr };
}

void Interrupt<RS485>::Status::register_callback(const Interrupt<RS485>::Status::Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()));
    bit_flag::set(&(p_registers->CR1), USART_CR1_PEIE);
    bit_flag::set(&(p_registers->CR3), USART_CR3_EIE);
}
void Interrupt<RS485>::Status::unregister_callback()
{
    Interrupt_guard guard;

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(this->p_owner->get_handle()));
    bit_flag::clear(&(p_registers->CR1), USART_CR1_PEIE);
    bit_flag::clear(&(p_registers->CR3), USART_CR3_EIE);

    this->callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif