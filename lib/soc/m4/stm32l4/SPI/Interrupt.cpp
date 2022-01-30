/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/SPI/Interrupt.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/SPI/common.hpp>

// cml
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void SPI_interrupt_handler(SPI_transmission_interrupt::TX* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    const std::uint32_t sr  = a_p_this->p_registers->SR;
    const std::uint32_t cr2 = a_p_this->p_registers->CR2;

    if (true == bit_flag::is(sr, SPI_SR_TXE) && true == bit_flag::is(cr2, SPI_CR2_TXEIE) &&
        nullptr != a_p_this->callback.function)
    {
        if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
            SPI_master::Frame_format::Word_length::_8)
        {
            a_p_this->callback.function(reinterpret_cast<volatile std::uint16_t*>(&(a_p_this->p_registers->DR)),
                                        false,
                                        a_p_this->callback.p_user_data);
        }
        else
        {
            a_p_this->callback.function(reinterpret_cast<volatile std::uint16_t*>(
                                            reinterpret_cast<volatile std::uint8_t*>(&(a_p_this->p_registers->DR))),
                                        false,
                                        a_p_this->callback.p_user_data);
        }

        if (false == bit_flag::is(a_p_this->p_registers->SR, SPI_SR_BSY))
        {
            a_p_this->callback.function(nullptr, true, a_p_this->callback.p_user_data);
        }
    }
}
void SPI_interrupt_handler(SPI_transmission_interrupt::RX* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    const std::uint32_t sr  = a_p_this->p_registers->SR;
    const std::uint32_t cr2 = a_p_this->p_registers->CR2;

    if (true == bit_flag::is(sr, SPI_SR_RXNE) && true == bit_flag::is(cr2, SPI_CR2_RXNEIE) &&
        nullptr != a_p_this->callback.function)
    {
        if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
            SPI_master::Frame_format::Word_length::_8)
        {
            a_p_this->callback.function((*reinterpret_cast<volatile std::uint16_t*>(&(a_p_this->p_registers->DR))),
                                        a_p_this->callback.p_user_data);
        }
        else
        {
            a_p_this->callback.function((*reinterpret_cast<volatile std::uint8_t*>(&(a_p_this->p_registers->DR))),
                                        a_p_this->callback.p_user_data);
        }
    }
}
void SPI_interrupt_handler(SPI_status_interrupt* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    const std::uint32_t sr = a_p_this->p_registers->SR;

    if (true == bit::is_any(sr, SPI_SR_FRE | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR) &&
        nullptr != a_p_this->callback.function)
    {
        a_p_this->callback.function(get_Bus_flag<SPI_status_interrupt::Callback::Flag>(a_p_this->p_registers),
                                    a_p_this->callback.p_user_data);

        clear_errors(a_p_this->p_registers);
    }
}

void SPI_status_interrupt::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;
    bit_flag::set(&(this->p_registers->CR2), SPI_CR2_ERRIE);
}

void SPI_status_interrupt::unregister_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(this->p_registers->CR2), SPI_CR2_ERRIE);
    this->callback = { nullptr, nullptr };
}

void SPI_transmission_interrupt::TX::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;
    cml::bit_flag::set(&(this->p_registers->CR2), SPI_CR2_TXEIE);
}

void SPI_transmission_interrupt::TX::unregister_callback()
{
    Interrupt_guard guard;

    cml::bit_flag::set(&(this->p_registers->CR2), SPI_CR2_TXEIE);

    this->callback = { nullptr, nullptr };
}

void SPI_transmission_interrupt::RX::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->callback = a_callback;
    cml::bit_flag::set(&(this->p_registers->CR2), SPI_CR2_RXNEIE);
}

void SPI_transmission_interrupt::RX::unregister_callback()
{
    Interrupt_guard guard;

    cml::bit_flag::set(&(this->p_registers->CR2), SPI_CR2_RXNEIE);

    this->callback = { nullptr, nullptr };
}

void Interrupt<SPI_master>::enable(const IRQ_config& a_irq_config)
{
    this->set_irq_context();

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);
}

void Interrupt<SPI_master>::disable()
{
    NVIC_DisableIRQ(this->irqn);

    this->clear_irq_context();
}

void Interrupt<SPI_slave>::enable(const IRQ_config& a_irq_config)
{
    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);
}

void Interrupt<SPI_slave>::disable()
{
    NVIC_DisableIRQ(this->irqn);
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif