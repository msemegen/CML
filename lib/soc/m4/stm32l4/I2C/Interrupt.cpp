/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/I2C/Interrupt.hpp>

// soc
#include <soc/m4/stm32l4/I2C/common.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void I2C_interrupt_handler(I2C_status_interrupt* a_p_this)
{
    if (true == is_error(a_p_this->p_registers))
    {
        const Interrupt<I2C_master>::Status::Callback::Flag status =
            get_Bus_status_flag<Interrupt<I2C_master>::Status::Callback::Flag>(a_p_this->p_registers);

        if (Interrupt<I2C_master>::Status::Callback::Flag::ok != status)
        {
            if (nullptr != a_p_this->callback.function)
            {
                a_p_this->callback.function(status, a_p_this->callback.p_user_data);
            }

            bit_flag::set(&(a_p_this->p_registers->ICR),
                          I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                              I2C_ICR_NACKCF);
        }
    }
}
void I2C_interrupt_handler(I2C_TX_interrupt* a_p_this)
{
    const std::uint32_t isr = a_p_this->p_registers->ISR;
    const std::uint32_t cr1 = a_p_this->p_registers->CR1;

    if (true == bit_flag::is(isr, I2C_ISR_TXE) && true == bit_flag::is(cr1, I2C_CR1_TXIE) &&
        nullptr != a_p_this->callback.function)
    {
        a_p_this->callback.function(reinterpret_cast<volatile std::uint32_t*>(&(a_p_this->p_registers->TXDR)),
                                    false,
                                    a_p_this->callback.p_user_data);
    }

    if (true == bit_flag::is(isr, I2C_ISR_STOPF) && true == bit_flag::is(cr1, I2C_CR1_STOPIE))
    {
        bit_flag::set(&(a_p_this->p_registers->ICR), I2C_ICR_STOPCF);

        if (nullptr != a_p_this->callback.function)
        {
            a_p_this->callback.function(nullptr, true, a_p_this->callback.p_user_data);
        }
    }
}
void I2C_interrupt_handler(I2C_RX_interrupt* a_p_this)
{
    const std::uint32_t isr = a_p_this->p_registers->ISR;
    const std::uint32_t cr1 = a_p_this->p_registers->CR1;

    if (true == bit_flag::is(isr, I2C_ISR_RXNE) && true == bit_flag::is(cr1, I2C_CR1_RXIE) &&
        nullptr != a_p_this->callback.function)
    {
        a_p_this->callback.function(
            static_cast<std::uint8_t>(a_p_this->p_registers->RXDR), false, a_p_this->callback.p_user_data);
    }

    if (true == bit_flag::is(isr, I2C_ISR_STOPF) && true == bit_flag::is(cr1, I2C_CR1_STOPIE))
    {
        bit_flag::set(&(a_p_this->p_registers->ICR), I2C_ICR_STOPCF);

        if (nullptr != a_p_this->callback.function)
        {
            a_p_this->callback.function(0, true, a_p_this->callback.p_user_data);
        }
    }
}

void I2C_interrupt_handler(Interrupt<I2C>* a_p_this)
{
    I2C_interrupt_handler(&(a_p_this->status));
    I2C_interrupt_handler(a_p_this->p_tx);
    I2C_interrupt_handler(a_p_this->p_rx);
}

void I2C_status_interrupt::enable(const IRQ_config& a_irq_config)
{
    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    this->set_irq_context();
}
void I2C_status_interrupt::disable()
{
    NVIC_DisableIRQ(this->irqn);

    this->clear_irq_context();
}

void I2C_status_interrupt::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;
    cml::bit_flag::set(&(this->p_registers->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);
}
void I2C_status_interrupt::unregister_callback()
{
    Interrupt_guard guard;

    cml::bit_flag::clear(&(this->p_registers->CR1), I2C_CR1_NACKIE | I2C_CR1_ERRIE);

    this->callback = { nullptr, nullptr };
}

void Interrupt<I2C_master>::Transmission::enable(const IRQ_config& a_irq_config)
{
    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    this->set_irq_context();
}

void Interrupt<I2C_master>::Transmission::disable()
{
    this->clear_irq_context();

    NVIC_DisableIRQ(this->irqn);
}

void Interrupt<I2C_master>::Transmission::TX::register_callback(std::uint8_t a_slave_address,
                                                                std::size_t a_data_length_in_bytes,
                                                                const Callback& a_callback)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    const uint32_t address_mask   = static_cast<uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_length_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND;
    bit_flag::set(&(this->p_registers->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);
}

void Interrupt<I2C_master>::Transmission::TX::unregister_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(this->p_registers->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);

    this->callback = { nullptr, nullptr };
}

void Interrupt<I2C_master>::Transmission::RX::register_callback(std::uint8_t a_slave_address,
                                                                std::size_t a_data_length_in_bytes,
                                                                const Callback& a_callback)
{
    cml_assert(a_slave_address <= 0xFE);
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    const uint32_t address_mask   = static_cast<uint32_t>(a_slave_address) & I2C_CR2_SADD;
    const uint32_t data_size_mask = static_cast<uint32_t>(a_data_length_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_registers->CR2 = address_mask | data_size_mask | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;
    bit_flag::set(&(this->p_registers->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);
}

void Interrupt<I2C_master>::Transmission::RX::unregister_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(this->p_registers->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE);

    this->callback = { nullptr, nullptr };
}

void Interrupt<I2C_slave>::Transmission::enable(const IRQ_config& a_irq_config)
{
    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    this->set_irq_context();
}

void Interrupt<I2C_slave>::Transmission::disable()
{
    NVIC_DisableIRQ(this->irqn);

    this->clear_irq_context();
}

void Interrupt<I2C_slave>::Transmission::TX::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    bit_flag::set(&(this->p_registers->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE | I2C_CR1_NACKIE);
}

void Interrupt<I2C_slave>::Transmission::TX::unregister_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(this->p_registers->CR1), I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE | I2C_CR1_NACKIE);

    this->callback = { nullptr, nullptr };
}

void Interrupt<I2C_slave>::Transmission::RX::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;

    bit_flag::set(&(this->p_registers->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);
}

void Interrupt<I2C_slave>::Transmission::RX::unregister_callback()
{
    Interrupt_guard guard;

    bit_flag::set(&(this->p_registers->CR1), I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE);

    this->callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif