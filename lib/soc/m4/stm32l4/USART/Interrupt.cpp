/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/RS485/Interrupt.hpp>
#include <soc/m4/stm32l4/USART/Interrupt.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/USART/common.hpp>

// cml
#include <cml/bit_flag.hpp>

namespace {

using namespace soc::m4::stm32l4;

enum class Mode : std::uint32_t
{
    USART,
    RS485,
    none
};

struct Controller
{
    union
    {
        void* p_general = nullptr;
        Interrupt<USART>* p_usart;
        Interrupt<RS485>* p_RS485;
    };

    Mode mode = Mode::none;
};

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
Controller controllers[3];
#endif
#if defined(STM32L432xx) || defined(STM32L442xx)
Controller controllers[2];
#endif

} // namespace

extern "C" {

void USART1_IRQHandler()
{
    cml_assert(nullptr != controllers[0].p_general);

    switch (controllers[0].mode)
    {
        case Mode::RS485: {
            RS485_interrupt_handler(controllers[0].p_RS485);
        }
        break;

        case Mode::USART: {
            usart_interrupt_handler(controllers[0].p_usart);
        }
        break;

        case Mode::none: {
            cml_assert(false);
        }
    }
}

void USART2_IRQHandler()
{
    cml_assert(nullptr != controllers[1].p_general);

    switch (controllers[1].mode)
    {
        case Mode::RS485: {
            RS485_interrupt_handler(controllers[1].p_RS485);
        }
        break;

        case Mode::USART: {
            usart_interrupt_handler(controllers[1].p_usart);
        }
        break;

        case Mode::none: {
            cml_assert(false);
        }
    }
}

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
void USART3_IRQHandler()
{
    cml_assert(nullptr != controllers[2].p_general && Mode::none != controllers[2].mode);

    switch (controllers[2].mode)
    {
        case Mode::RS485: {
            RS485_interrupt_handler(controllers[2].p_RS485);
        }
        break;

        case Mode::USART: {
            usart_interrupt_handler(controllers[2].p_usart);
        }
        break;

        case Mode::none: {
            cml_assert(false);
        }
    }
}
#endif
}

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void usart_interrupt_handler(Interrupt<USART>* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(a_p_this->p_usart));

    const std::uint32_t isr = p_registers->ISR;
    const std::uint32_t cr1 = p_registers->CR1;
    const std::uint32_t cr3 = p_registers->CR3;

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_TXE) && true == bit_flag::is(cr1, USART_CR1_TXEIE))
        {
            a_p_this->transmit_callback.function(
                &(p_registers->TDR), false, a_p_this->p_usart, a_p_this->transmit_callback.p_user_data);
        }

        if (true == bit_flag::is(isr, USART_ISR_TC) && true == bit_flag::is(cr1, USART_CR1_TCIE))
        {
            a_p_this->transmit_callback.function(
                nullptr, true, a_p_this->p_usart, a_p_this->transmit_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_RXNE) && true == bit_flag::is(cr1, USART_CR1_RXNEIE))
        {
            a_p_this->receive_callback.function(
                p_registers->RDR, false, a_p_this->p_usart, a_p_this->receive_callback.p_user_data);
        }
        else if (true == bit_flag::is(isr, USART_ISR_IDLE) && true == bit_flag::is(cr1, USART_CR1_IDLEIE))
        {
            bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);
            a_p_this->receive_callback.function(0x0u, true, a_p_this->p_usart, a_p_this->receive_callback.p_user_data);
        }
    }

    if (true == bit_flag::is(cr3, USART_CR3_EIE) && true == bit_flag::is(cr1, USART_CR1_PEIE))
    {
        Interrupt<USART>::Bus_status_callback::Status_flag status =
            get_Bus_status_flag<Interrupt<USART>::Bus_status_callback::Status_flag>(p_registers);

        if (status != Interrupt<USART>::Bus_status_callback::Status_flag::ok)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(
                    status, a_p_this->p_usart, a_p_this->bus_status_callback.p_user_data);
            }

            bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
        }
    }
}

void RS485_interrupt_handler(Interrupt<RS485>* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    USART_TypeDef* p_registers = static_cast<USART_TypeDef*>(*(a_p_this->p_RS485));

    const std::uint32_t isr = p_registers->ISR;
    const std::uint32_t cr1 = p_registers->CR1;
    const std::uint32_t cr3 = p_registers->CR3;

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_TXE) && true == bit_flag::is(cr1, USART_CR1_TXEIE))
        {
            a_p_this->transmit_callback.function(
                &(p_registers->TDR), false, a_p_this->p_RS485, a_p_this->transmit_callback.p_user_data);
        }

        if (true == bit_flag::is(isr, USART_ISR_TC) && true == bit_flag::is(cr1, USART_CR1_TCIE))
        {
            a_p_this->transmit_callback.function(
                nullptr, true, a_p_this->p_RS485, a_p_this->transmit_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_RXNE) && true == bit_flag::is(cr1, USART_CR1_RXNEIE))
        {
            volatile uint16_t rdr = p_registers->RDR;

            if (false == bit_flag::is(rdr, 0x100))
            {
                a_p_this->receive_callback.function(
                    rdr, false, a_p_this->p_RS485, a_p_this->receive_callback.p_user_data);
            }
        }
        else if (true == bit_flag::is(isr, USART_ISR_IDLE) && true == bit_flag::is(cr1, USART_CR1_IDLEIE))
        {
            bit_flag::set(&(p_registers->ICR), USART_ICR_IDLECF);
            a_p_this->receive_callback.function(0x0u, true, a_p_this->p_RS485, a_p_this->receive_callback.p_user_data);
        }
    }

    if (true == bit_flag::is(cr3, USART_CR3_EIE) && true == bit_flag::is(cr1, USART_CR1_PEIE))
    {
        Interrupt<RS485>::Bus_status_callback::Status_flag status =
            get_Bus_status_flag<Interrupt<RS485>::Bus_status_callback::Status_flag>(p_registers);

        if (status != Interrupt<RS485>::Bus_status_callback::Status_flag::ok)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(
                    status, a_p_this->p_RS485, a_p_this->bus_status_callback.p_user_data);
            }

            bit_flag::set(&(p_registers->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
        }
    }
}

void Interrupt<USART>::enable(const IRQ& a_irq)
{
    cml_assert(true == a_irq.active);

    NVIC_SetPriority(this->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq.preempt_priority, a_irq.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    controllers[this->p_usart->get_id()] = { this, Mode::USART };
}

void Interrupt<USART>::disable()
{
    NVIC_DisableIRQ(static_cast<IRQn_Type>(this->irqn));

    controllers[this->p_usart->get_id()] = { nullptr, Mode::none };
}

void Interrupt<USART>::register_callback(const Transmit_callback& a_callback)
{
    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    if (nullptr != this->transmit_callback.function)
    {
        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_usart))->ICR), USART_ICR_TCCF);
        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_usart))->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_usart))->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);
    }
}

void Interrupt<USART>::register_callback(const Receive_callback& a_callback)
{
    Interrupt_guard guard;

    this->receive_callback = a_callback;

    if (nullptr != this->receive_callback.function)
    {
        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_usart))->ICR), USART_ICR_IDLECF);
        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_usart))->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_usart))->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
    }
}

void Interrupt<USART>::register_callback(const Bus_status_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->bus_status_callback = a_callback;

        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_usart))->CR1), USART_CR1_PEIE);
        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_usart))->CR3), USART_CR3_EIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_usart))->CR1), USART_CR1_PEIE);
        bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_usart))->CR3), USART_CR3_EIE);

        this->bus_status_callback = a_callback;
    }
}

void Interrupt<RS485>::enable(const IRQ& a_irq)
{
    cml_assert(true == a_irq.active);

    NVIC_SetPriority(this->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq.preempt_priority, a_irq.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    controllers[this->p_RS485->get_id()] = { this, Mode::RS485 };
}

void Interrupt<RS485>::disable()
{
    NVIC_DisableIRQ(static_cast<IRQn_Type>(this->irqn));

    controllers[this->p_RS485->get_id()] = { nullptr, Mode::none };
}

void Interrupt<RS485>::register_callback(const Transmit_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->transmit_callback = a_callback;

        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->ICR), USART_ICR_TCCF);
        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);

        this->p_flow_control_pin->set_level(GPIO::Level::high);
    }
    else
    {
        this->p_flow_control_pin->set_level(GPIO::Level::low);
        bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);

        this->transmit_callback = { nullptr, nullptr };
    }
}

void Interrupt<RS485>::register_callback(const Receive_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->receive_callback = a_callback;

        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->ICR), USART_ICR_IDLECF);
        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
    }
    else
    {
        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->ICR), USART_ICR_CMCF);
        bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR1), USART_CR1_RXNEIE);

        this->receive_callback = { nullptr, nullptr };
    }
}

void Interrupt<RS485>::register_callback(const Bus_status_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->bus_status_callback = a_callback;

        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR1), USART_CR1_PEIE);
        bit_flag::set(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR3), USART_CR3_EIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR1), USART_CR1_PEIE);
        bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_RS485))->CR3), USART_CR3_EIE);

        this->bus_status_callback = { nullptr, nullptr };
    }
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif