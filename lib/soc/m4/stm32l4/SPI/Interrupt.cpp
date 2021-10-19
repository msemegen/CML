/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/SPI/Interrupt.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/SPI/common.hpp>

// cml
#include <cml/bit_flag.hpp>

namespace {

using namespace cml;
using namespace soc::m4::stm32l4;

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L432xx) || defined(STM32L442xx)
SPI_interrupt* spis[2] = { nullptr, nullptr };
#endif

#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
SPI_interrupt* spis[3] = { nullptr, nullptr, nullptr };
#endif

} // namespace

extern "C" {
#if defined(STM32L412xx) || defined(STM32L422xx)
void SPI1_IRQHandler()
{
    cml_assert(nullptr != spis[0]);
    spi_interrupt_handler(spis[0]);
}
void SPI2_IRQHandler()
{
    cml_assert(nullptr != spis[1]);
    spi_interrupt_handler(spis[1]);
}
#endif

#if defined(STM32L432xx) || defined(STM32L442xx)
void SPI1_IRQHandler()
{
    cml_assert(nullptr != spis[0]);
    spi_interrupt_handler(spis[0]);
}
void SPI3_IRQHandler()
{
    cml_assert(nullptr != spis[1]);
    spi_interrupt_handler(spis[1]);
}
#endif

#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
void SPI1_IRQHandler()
{
    cml_assert(nullptr != spis[0]);
    spi_interrupt_handler(spis[0]);
}
void SPI2_IRQHandler()
{
    cml_assert(nullptr != spis[1]);
    spi_interrupt_handler(spis[1]);
}
void SPI3_IRQHandler()
{
    cml_assert(nullptr != spis[2]);
    spi_interrupt_handler(spis[2]);
}
#endif
}

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void spi_interrupt_handler(SPI_interrupt* a_p_this)
{
    SPI_TypeDef* p_registers = static_cast<SPI_TypeDef*>(*(a_p_this->p_SPI));

    const std::uint32_t sr  = p_registers->SR;
    const std::uint32_t cr2 = p_registers->CR2;

    if (true == bit::is_any(sr, SPI_SR_FRE | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR))
    {
        if (nullptr != a_p_this->bus_status_callback.function)
        {
            a_p_this->bus_status_callback.function(get_Bus_flag<Interrupt<SPI_master>::Bus_flag>(p_registers),
                                                   a_p_this->p_SPI,
                                                   a_p_this->transmit_callback.p_user_data);
        }

        clear_errors(p_registers);
    }

    if (nullptr != a_p_this->transmit_callback.function || nullptr != a_p_this->transmit_receive_callback.transmit)
    {
        if (true == bit_flag::is(sr, SPI_SR_TXE) && true == bit_flag::is(cr2, SPI_CR2_TXEIE))
        {
            if (nullptr != a_p_this->transmit_callback.function)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
                    SPI_master::Frame_format::Word_length::_8)
                {
                    a_p_this->transmit_callback.function(reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR)),
                                                         false,
                                                         a_p_this->p_SPI,
                                                         a_p_this->transmit_callback.p_user_data);
                }
                else
                {
                    a_p_this->transmit_callback.function(
                        reinterpret_cast<volatile std::uint16_t*>(
                            reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))),
                        false,
                        a_p_this->p_SPI,
                        a_p_this->transmit_callback.p_user_data);
                }
            }

            if (nullptr != a_p_this->transmit_receive_callback.transmit)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
                    SPI_master::Frame_format::Word_length::_8)
                {
                    a_p_this->transmit_receive_callback.transmit(
                        reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR)),
                        false,
                        a_p_this->p_SPI,
                        a_p_this->transmit_receive_callback.p_user_data);
                }
                else
                {
                    a_p_this->transmit_receive_callback.transmit(
                        reinterpret_cast<volatile std::uint16_t*>(
                            reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))),
                        false,
                        a_p_this->p_SPI,
                        a_p_this->transmit_receive_callback.p_user_data);
                }
            }

            if (false == bit_flag::is(p_registers->SR, SPI_SR_BSY))
            {
                if (nullptr != a_p_this->transmit_callback.function)
                {
                    a_p_this->transmit_callback.function(
                        nullptr, true, a_p_this->p_SPI, a_p_this->transmit_callback.p_user_data);
                }

                if (nullptr != a_p_this->transmit_receive_callback.transmit)
                {
                    a_p_this->transmit_receive_callback.transmit(
                        nullptr, true, a_p_this->p_SPI, a_p_this->transmit_receive_callback.p_user_data);
                }
            }
        }
    }

    if (nullptr != a_p_this->receive_callback.function || nullptr != a_p_this->transmit_receive_callback.receive)
    {
        if (true == bit_flag::is(sr, SPI_SR_RXNE) && true == bit_flag::is(cr2, SPI_CR2_RXNEIE))
        {
            if (nullptr != a_p_this->receive_callback.function)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
                    SPI_master::Frame_format::Word_length::_8)
                {
                    a_p_this->receive_callback.function(
                        (*reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))),
                        a_p_this->p_SPI,
                        a_p_this->receive_callback.p_user_data);
                }
                else
                {
                    a_p_this->receive_callback.function((*reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))),
                                                        a_p_this->p_SPI,
                                                        a_p_this->receive_callback.p_user_data);
                }
            }

            if (nullptr != a_p_this->transmit_receive_callback.receive)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
                    SPI_master::Frame_format::Word_length::_8)
                {
                    a_p_this->transmit_receive_callback.receive(
                        (*reinterpret_cast<volatile std::uint16_t*>(&(p_registers->DR))),
                        a_p_this->p_SPI,
                        a_p_this->transmit_receive_callback.p_user_data);
                }
                else
                {
                    a_p_this->transmit_receive_callback.receive(
                        (*reinterpret_cast<volatile std::uint8_t*>(&(p_registers->DR))),
                        a_p_this->p_SPI,
                        a_p_this->transmit_receive_callback.p_user_data);
                }
            }
        }
    }
}

void SPI_interrupt::enable(const IRQ& a_irq)
{
    cml_assert(true == a_irq.active);

    NVIC_SetPriority(this->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq.preempt_priority, a_irq.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    spis[this->p_SPI->get_id()] = this;
}
void SPI_interrupt::disable()
{
    NVIC_DisableIRQ(this->irqn);

    spis[this->p_SPI->get_id()] = nullptr;
}

void SPI_interrupt::register_callback(const Transmit_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->transmit_callback = a_callback;
        bit_flag::set(&(static_cast<SPI_TypeDef*>(*(this->p_SPI))->CR2), SPI_CR2_TXEIE);
    }
    else
    {
        bit_flag::set(&(static_cast<SPI_TypeDef*>(*(this->p_SPI))->CR2), SPI_CR2_TXEIE);
        this->transmit_callback = { nullptr, nullptr };
    }
}

void SPI_interrupt::register_callback(const Receive_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->receive_callback = a_callback;
        bit_flag::set(&(static_cast<SPI_TypeDef*>(*(this->p_SPI))->CR2), SPI_CR2_RXNEIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<SPI_TypeDef*>(*(this->p_SPI))->CR2), SPI_CR2_RXNEIE);
        this->receive_callback = { nullptr, nullptr };
    }
}

void SPI_interrupt::register_callback(const Transmit_receive_callback& a_callback)
{
    cml_assert((nullptr == a_callback.receive && nullptr == a_callback.transmit) ||
               (nullptr != a_callback.receive && nullptr != a_callback.transmit));

    Interrupt_guard guard;

    if (nullptr != a_callback.receive && nullptr != a_callback.transmit)
    {
        this->transmit_receive_callback = a_callback;
        bit_flag::set(&(static_cast<SPI_TypeDef*>(*(this->p_SPI))->CR2), SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<SPI_TypeDef*>(*(this->p_SPI))->CR2), SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
        this->transmit_receive_callback = { nullptr, nullptr, nullptr };
    }
}

void SPI_interrupt::register_callback(const Bus_status_callback& a_callback)
{
    Interrupt_guard guard;

    if (nullptr != a_callback.function)
    {
        this->bus_status_callback = a_callback;
        bit_flag::set(&(static_cast<SPI_TypeDef*>(*(this->p_SPI))->CR2), SPI_CR2_ERRIE);
    }
    else
    {
        bit_flag::clear(&(static_cast<SPI_TypeDef*>(*(this->p_SPI))->CR2), SPI_CR2_ERRIE);
        this->bus_status_callback = { nullptr, nullptr };
    }
}

} // namespace stm32l4
} // namespace m4
} // namespace soc