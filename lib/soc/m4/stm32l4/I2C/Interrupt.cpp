/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/I2C/Interrupt.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/I2C/common.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/various.hpp>

namespace {

using namespace cml;
using namespace soc::m4::stm32l4;

struct Controller
{
    enum class Type
    {
        master,
        slave,
        none
    };

    void* p_I2C = nullptr;
    Type type   = Type::none;
};

#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
Controller controllers[4];
#endif

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx)
Controller controllers[3];
#endif

#if defined(STM32L432xx) || defined(STM32L442xx)
Controller controllers[2];
#endif

} // namespace

extern "C" {

using namespace soc::m4::stm32l4;

void event_interrupt_handler(std::uint32_t a_controller_index)
{
    switch (controllers[a_controller_index].type)
    {
        case Controller::Type::master: {
            i2c_master_event_interrupt_handler(
                static_cast<Interrupt<I2C_master>*>(controllers[a_controller_index].p_I2C));
        }
        break;

        case Controller::Type::slave: {
            i2c_slave_event_interrupt_handler(
                static_cast<Interrupt<I2C_slave>*>(controllers[a_controller_index].p_I2C));
        }
        break;

        case Controller::Type::none: {
            cml_assert(false);
        }
    }
}

void bus_status_interrupt_handler(std::uint32_t a_controller_index)
{
    switch (controllers[a_controller_index].type)
    {
        case Controller::Type::master: {
            i2c_master_bus_status_interrupt_handler(
                static_cast<Interrupt<I2C_master>*>(controllers[a_controller_index].p_I2C));
        }
        break;

        case Controller::Type::slave: {
            i2c_slave_bus_status_interrupt_handler(
                static_cast<Interrupt<I2C_slave>*>(controllers[a_controller_index].p_I2C));
        }
        break;

        case Controller::Type::none: {
            cml_assert(false);
        }
    }
}

void I2C1_EV_IRQHandler()
{
    event_interrupt_handler(0);
}

void I2C1_ER_IRQHandler()
{
    bus_status_interrupt_handler(0);
}

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
void I2C2_EV_IRQHandler()
{
    event_interrupt_handler(1);
}

void I2C2_ER_IRQHandler()
{
    bus_status_interrupt_handler(1);
}
#endif

void I2C3_EV_IRQHandler()
{
    event_interrupt_handler(2);
}

void I2C3_ER_IRQHandler()
{
    bus_status_interrupt_handler(2);
}

#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
void event_I2C4_EV_IRQHandler()
{
    event_interrupt_handler(3);
}

void I2C4_ER_IRQHandler()
{
    bus_status_interrupt_handler(3);
}

#endif

} // extern "C"

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void i2c_master_event_interrupt_handler(Interrupt<I2C_master>* a_p_this)
{
    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(a_p_this->p_I2C));

    const std::uint32_t isr = p_registers->ISR;
    const std::uint32_t cr1 = p_registers->CR1;

    if (nullptr != a_p_this->receive_event_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_RXNE) && true == bit_flag::is(cr1, I2C_CR1_RXIE))
        {
            a_p_this->receive_event_callback.function(static_cast<std::uint8_t>(p_registers->RXDR),
                                                      false,
                                                      a_p_this->p_I2C,
                                                      a_p_this->receive_event_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->transmit_event_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_TXE) && true == bit_flag::is(cr1, I2C_CR1_TXIE))
        {
            a_p_this->transmit_event_callback.function(reinterpret_cast<volatile std::uint32_t*>(&(p_registers->TXDR)),
                                                       false,
                                                       a_p_this->p_I2C,
                                                       a_p_this->transmit_event_callback.p_user_data);
        }
    }

    if (true == bit_flag::is(isr, I2C_ISR_STOPF) && true == bit_flag::is(cr1, I2C_CR1_STOPIE))
    {
        bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);

        if (nullptr != a_p_this->receive_event_callback.function)
        {
            a_p_this->receive_event_callback.function(
                0, true, a_p_this->p_I2C, a_p_this->receive_event_callback.p_user_data);
        }

        if (nullptr != a_p_this->transmit_event_callback.function)
        {
            a_p_this->transmit_event_callback.function(
                nullptr, true, a_p_this->p_I2C, a_p_this->transmit_event_callback.p_user_data);
        }
    }
}
void i2c_master_bus_status_interrupt_handler(Interrupt<I2C_master>* a_p_this)
{
    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(a_p_this->p_I2C));

    if (true == is_error(p_registers))
    {
        const Interrupt<I2C_master>::Bus_status_flag status =
            get_Bus_status_flag<Interrupt<I2C_master>::Bus_status_flag>(p_registers);

        if (Interrupt<I2C_master>::Bus_status_flag::ok != status)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(
                    status, a_p_this->p_I2C, a_p_this->bus_status_callback.p_user_data);
            }

            bit_flag::set(&(p_registers->ICR),
                          I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                              I2C_ICR_NACKCF);
        }
    }
}

void i2c_slave_event_interrupt_handler(Interrupt<I2C_slave>* a_p_this)
{
    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(a_p_this->p_I2C));

    const std::uint32_t isr = p_registers->ISR;
    const std::uint32_t cr1 = p_registers->CR1;

    if (nullptr != a_p_this->receive_event_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_RXNE) && true == bit_flag::is(cr1, I2C_CR1_RXIE))
        {
            a_p_this->receive_event_callback.function(static_cast<std::uint8_t>(p_registers->RXDR),
                                                      false,
                                                      a_p_this->p_I2C,
                                                      a_p_this->receive_event_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->transmit_event_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_TXE) && true == bit_flag::is(cr1, I2C_CR1_TXIE))
        {
            a_p_this->transmit_event_callback.function(reinterpret_cast<volatile std::uint32_t*>(&(p_registers->TXDR)),
                                                       false,
                                                       a_p_this->p_I2C,
                                                       a_p_this->transmit_event_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->address_match_event_callback.function)
    {
        if (true == bit_flag::is(isr, I2C_ISR_ADDR) && true == bit_flag::is(cr1, I2C_CR1_ADDRIE))
        {
            a_p_this->address_match_event_callback.function(a_p_this->p_I2C,
                                                            a_p_this->address_match_event_callback.p_user_data);

            bit_flag::set(&(p_registers->ICR), I2C_ICR_ADDRCF);
        }
    }

    if (true == bit_flag::is(isr, I2C_ISR_STOPF) && true == bit_flag::is(cr1, I2C_CR1_STOPIE))
    {
        bit_flag::set(&(p_registers->ICR), I2C_ICR_STOPCF);

        if (nullptr != a_p_this->receive_event_callback.function)
        {
            a_p_this->receive_event_callback.function(
                0, true, a_p_this->p_I2C, a_p_this->receive_event_callback.p_user_data);
        }

        if (nullptr != a_p_this->transmit_event_callback.function)
        {
            a_p_this->transmit_event_callback.function(
                nullptr, true, a_p_this->p_I2C, a_p_this->transmit_event_callback.p_user_data);
        }
    }
}
void i2c_slave_bus_status_interrupt_handler(Interrupt<I2C_slave>* a_p_this)
{
    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(a_p_this->p_I2C));

    if (true == bit_flag::is(p_registers->ISR, I2C_ISR_NACKF) && nullptr != a_p_this->transmit_event_callback.function)
    {
        bit_flag::set(&(p_registers->ICR), I2C_ICR_NACKCF);
    }
    else
    {
        const Interrupt<I2C_slave>::Bus_status_flag status =
            get_Bus_status_flag<Interrupt<I2C_slave>::Bus_status_flag>(p_registers);

        if (Interrupt<I2C_slave>::Bus_status_flag::ok != status)
        {
            if (nullptr != a_p_this->bus_status_callback.function)
            {
                a_p_this->bus_status_callback.function(
                    status, a_p_this->p_I2C, a_p_this->bus_status_callback.p_user_data);
            }

            bit_flag::set(&(p_registers->ICR),
                          I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                              I2C_ICR_NACKCF);
        }
    }
}

void Interrupt<I2C_master>::enable(const IRQ& a_event_irq, const IRQ& a_bus_status_irq)
{
    controllers[this->p_I2C->get_id()].p_I2C = this;
    controllers[this->p_I2C->get_id()].type  = Controller::Type::master;

    if (true == a_event_irq.active)
    {
        NVIC_SetPriority(
            this->irqn_ev,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_event_irq.preempt_priority, a_event_irq.sub_priority));
        NVIC_EnableIRQ(this->irqn_ev);
    }

    if (true == a_event_irq.active)
    {
        NVIC_SetPriority(this->irqn_er,
                         NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                                             a_bus_status_irq.preempt_priority,
                                             a_bus_status_irq.sub_priority));
        NVIC_EnableIRQ(this->irqn_er);
    }
}

void Interrupt<I2C_master>::disable()
{
    controllers[this->p_I2C->get_id()].p_I2C = nullptr;
    controllers[this->p_I2C->get_id()].type  = Controller::Type::none;

    NVIC_DisableIRQ(this->irqn_ev);
    NVIC_DisableIRQ(this->irqn_er);
}

void Interrupt<I2C_master>::register_callback(const Transmit_event_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->p_I2C));

    if (nullptr != a_callback.function)
    {
        cml_assert(a_callback.slave_address <= 0xFEu);
        cml_assert(a_callback.data_length_in_bytes < 0xFFu);

        this->transmit_event_callback = a_callback;

        const std::uint32_t address_mask   = static_cast<std::uint32_t>(a_callback.slave_address) & I2C_CR2_SADD;
        const std::uint32_t data_size_mask = static_cast<std::uint32_t>(a_callback.data_length_in_bytes)
                                             << I2C_CR2_NBYTES_Pos;

        p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;
        bit_flag::set(&(p_registers->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);
    }
    else
    {
        bit_flag::clear(&(p_registers->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);
        this->transmit_event_callback = { nullptr, nullptr, 0xFFu, 0xFFu };
    }
}

void Interrupt<I2C_master>::register_callback(const Receive_event_callback& a_callback)
{
    Interrupt_guard guard;

    I2C_TypeDef* p_registers = static_cast<I2C_TypeDef*>(*(this->p_I2C));

    if (nullptr != this->receive_event_callback.function)
    {
        cml_assert(a_callback.slave_address <= 0xFEu);
        cml_assert(a_callback.data_length_in_bytes < 0xFFu);

        this->receive_event_callback = a_callback;

        const std::uint32_t address_mask   = static_cast<std::uint32_t>(a_callback.slave_address) & I2C_CR2_SADD;
        const std::uint32_t data_size_mask = static_cast<std::uint32_t>(a_callback.data_length_in_bytes)
                                             << I2C_CR2_NBYTES_Pos;

        p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
        bit_flag::set(&(p_registers->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);
    }
    else
    {
        bit_flag::clear(&(p_registers->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE);
        this->receive_event_callback = { nullptr, nullptr, 0xFFu, 0xFFu };
    }
}

void Interrupt<I2C_master>::register_callback(const Bus_status_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        cml_assert(a_callback.slave_address <= 0xFEu);

        this->bus_status_callback = a_callback;
        bit_flag::set(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);
        this->receive_event_callback = { nullptr, nullptr, 0xFFu };
    }
}

void Interrupt<I2C_slave>::enable(const IRQ& a_event_irq, const IRQ& a_bus_status_irq)
{
    controllers[this->p_I2C->get_id()].p_I2C = this->p_I2C;
    controllers[this->p_I2C->get_id()].type  = Controller::Type::slave;

    if (true == a_event_irq.active)
    {
        NVIC_SetPriority(
            this->irqn_ev,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_event_irq.preempt_priority, a_event_irq.sub_priority));
        NVIC_EnableIRQ(this->irqn_ev);
    }

    if (true == a_event_irq.active)
    {
        NVIC_SetPriority(this->irqn_er,
                         NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                                             a_bus_status_irq.preempt_priority,
                                             a_bus_status_irq.sub_priority));
        NVIC_EnableIRQ(this->irqn_er);
    }
}

void Interrupt<I2C_slave>::disable()
{
    controllers[this->p_I2C->get_id()].p_I2C = nullptr;
    controllers[this->p_I2C->get_id()].type  = Controller::Type::none;

    NVIC_DisableIRQ(this->irqn_ev);
    NVIC_DisableIRQ(this->irqn_er);
}

void Interrupt<I2C_slave>::register_callback(const Transmit_event_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->transmit_event_callback = a_callback;

        bit_flag::set(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1),
                      I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE | I2C_CR1_NACKIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1),
                        I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE | I2C_CR1_NACKIE);

        this->transmit_event_callback = { nullptr, nullptr, 0xFFu };
    }
}

void Interrupt<I2C_slave>::register_callback(const Receive_event_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        cml_assert(a_callback.data_length_in_bytes <= 0xFFu);

        this->receive_event_callback = a_callback;

        bit_flag::set(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1),
                      I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1),
                        I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);

        this->receive_event_callback = { nullptr, nullptr, 0xFFu };
    }
}

void Interrupt<I2C_slave>::register_callback(const Address_match_event_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->address_match_event_callback = a_callback;

        bit_flag::set(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1), I2C_CR1_ADDRIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1), I2C_CR1_ADDRIE);

        this->address_match_event_callback = { nullptr, nullptr };
    }
}

void Interrupt<I2C_slave>::register_callback(const Bus_status_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->bus_status_callback = a_callback;

        bit_flag::set(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1),
                      I2C_CR1_NACKIE | I2C_CR1_ADDRIE | I2C_CR1_ERRIE);
    }
    else
    {
        this->bus_status_callback = a_callback;

        bit_flag::clear(&(static_cast<I2C_TypeDef*>(*(this->p_I2C))->CR1),
                        I2C_CR1_NACKIE | I2C_CR1_ADDRIE | I2C_CR1_ERRIE);
    }
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif
