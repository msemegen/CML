/*
 *   Name: DMA.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/USART/DMA.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {
using namespace cml;
using namespace soc::m4::stm32l4;

void transmit(USART_TypeDef* a_p_USART_registers,
              DMA_Request_TypeDef* a_p_request_registers,
              DMA_Channel_TypeDef* a_p_channel_registers,
              std::uint32_t a_channel_flags,
              const DMA<>::CSELR& a_cselr,
              const void* a_p_buffer,
              std::uint16_t a_buffer_size_in_words)
{
    cml_assert(false == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_EN));

    a_p_channel_registers->CNDTR = a_buffer_size_in_words;
    a_p_channel_registers->CPAR  = reinterpret_cast<std::uint32_t>(&(a_p_USART_registers->TDR));
    a_p_channel_registers->CMAR  = reinterpret_cast<std::uint32_t>(a_p_buffer);

    bit_flag::set(&(a_p_USART_registers->CR3), USART_CR3_DMAT);
    bit_flag::set(&(a_p_USART_registers->ICR), USART_ICR_TCCF);

    bit_flag::set(&(a_p_request_registers->CSELR), a_cselr.clear, a_cselr.set);
    bit_flag::set(&(a_p_channel_registers->CCR), a_channel_flags | DMA_CCR_EN);
}

void receive(USART_TypeDef* a_p_USART_registers,
             DMA_Request_TypeDef* a_p_request_registers,
             DMA_Channel_TypeDef* a_p_channel_registers,
             std::uint32_t a_channel_flags,
             const DMA<>::CSELR& a_cselr,
             void* a_p_buffer,
             std::uint16_t a_buffer_size_in_words)
{
    cml_assert(false == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_EN));

    a_p_channel_registers->CNDTR = a_buffer_size_in_words;
    a_p_channel_registers->CPAR  = reinterpret_cast<std::uint32_t>(&(a_p_USART_registers->RDR));
    a_p_channel_registers->CMAR  = reinterpret_cast<std::uint32_t>(a_p_buffer);

    bit_flag::set(&(a_p_USART_registers->CR3), USART_CR3_DMAR);

    bit_flag::set(&(a_p_request_registers->CSELR), a_cselr.clear, a_cselr.set);
    bit_flag::set(&(a_p_channel_registers->CCR), a_channel_flags | DMA_CCR_EN);
}
} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void DMA<USART>::transmit_start(DMA<>::Priority a_priority,
                                DMA<>::Mode a_mode,
                                const void* a_p_buffer,
                                std::uint16_t a_buffer_size_in_words)
{
    transmit(this->p_USART_registers,
             this->p_request_registers,
             this->p_tx_channel_registers,
             static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                 (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) | DMA_CCR_DIR |
                 (USART_CR1_M0 == bit_flag::get(this->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                      (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                      0x0u),
             this->tx_cselr,
             a_p_buffer,
             a_buffer_size_in_words);
}

void DMA<USART>::transmit_start(DMA<>::Priority a_priority,
                                DMA<>::Mode a_mode,
                                const void* a_p_buffer,
                                std::uint16_t a_buffer_size_in_words,
                                const IRQ_config& a_irq_config,
                                const DMA<>::Callback& a_callback,
                                DMA<>::Event_flag a_flag)
{
    cml_assert(nullptr != a_callback.function);

    this->tx_callback = a_callback;

    this->set_tx_irq_context();

    NVIC_SetPriority(
        this->tx_irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->tx_irqn);

    bit_flag::set(&(this->p_tx_channel_registers->CCR), static_cast<std::uint32_t>(a_flag));

    this->transmit_start(a_priority, a_mode, a_p_buffer, a_buffer_size_in_words);
}

void DMA<USART>::transmit_stop()
{
    bit_flag::clear(&(this->p_USART_registers->CR3), USART_CR3_DMAT);
    bit_flag::clear(&(this->p_request_registers->CSELR), this->tx_cselr.clear);

    this->p_tx_channel_registers->CCR   = 0x0u;
    this->p_tx_channel_registers->CNDTR = 0x0u;
    this->p_tx_channel_registers->CPAR  = 0x0u;
    this->p_tx_channel_registers->CMAR  = 0x0u;

    NVIC_DisableIRQ(this->tx_irqn);
    this->tx_callback = { nullptr, nullptr };

    if (nullptr != this->tx_callback.function)
    {
        this->clear_irq_context();
    }
}

void DMA<USART>::receive_start(DMA<>::Priority a_priority,
                               DMA<>::Mode a_mode,
                               void* a_p_buffer,
                               std::uint16_t a_buffer_size_in_words)
{
    receive(this->p_USART_registers,
            this->p_request_registers,
            this->p_rx_channel_registers,
            static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) |
                (USART_CR1_M0 == bit_flag::get(this->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                     (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                     0x0u),
            this->rx_cselr,
            a_p_buffer,
            a_buffer_size_in_words);
}

void DMA<USART>::receive_start(DMA<>::Priority a_priority,
                               DMA<>::Mode a_mode,
                               void* a_p_buffer,
                               std::uint16_t a_buffer_size_in_words,
                               const IRQ_config& a_irq_config,
                               const DMA<>::Callback& a_callback,
                               DMA<>::Event_flag a_flag)
{
    cml_assert(nullptr != a_callback.function);

    this->rx_callback = a_callback;

    this->set_rx_irq_context();

    NVIC_SetPriority(
        this->rx_irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->rx_irqn);

    bit_flag::set(&(this->p_rx_channel_registers->CCR), static_cast<std::uint32_t>(a_flag));

    this->receive_start(a_priority, a_mode, a_p_buffer, a_buffer_size_in_words);
}

void DMA<USART>::receive_stop()
{
    bit_flag::clear(&(this->p_USART_registers->CR3), USART_CR3_DMAR);
    bit_flag::clear(&(this->p_request_registers->CSELR), this->rx_cselr.clear);

    this->p_rx_channel_registers->CCR   = 0x0u;
    this->p_rx_channel_registers->CNDTR = 0x0u;
    this->p_rx_channel_registers->CPAR  = 0x0u;
    this->p_rx_channel_registers->CMAR  = 0x0u;

    NVIC_DisableIRQ(this->rx_irqn);

    if (nullptr != this->rx_callback.function)
    {
        this->clear_irq_context();
    }

    this->rx_callback = { nullptr, nullptr };
}

void DMA<RS485>::transmit_start(DMA<>::Priority a_priority,
                                DMA<>::Mode a_mode,
                                const void* a_p_buffer,
                                std::uint16_t a_buffer_size_in_words)
{
    transmit(this->p_RS485_registers,
             this->p_request_registers,
             this->p_tx_channel_registers,
             static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                 (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) | DMA_CCR_DIR |
                 (USART_CR1_M0 == bit_flag::get(this->p_RS485_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                      (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                      0x0u),
             this->tx_cselr,
             a_p_buffer,
             a_buffer_size_in_words);
}

void DMA<RS485>::transmit_start(DMA<>::Priority a_priority,
                                DMA<>::Mode a_mode,
                                const void* a_p_buffer,
                                std::uint16_t a_buffer_size_in_words,
                                const IRQ_config& a_irq_config,
                                const DMA<>::Callback& a_callback,
                                DMA<>::Event_flag a_flag)
{
    cml_assert(nullptr != a_callback.function);

    this->tx_callback = a_callback;

    this->set_tx_irq_context();

    NVIC_SetPriority(
        this->tx_irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->tx_irqn);

    bit_flag::set(&(this->p_tx_channel_registers->CCR), static_cast<std::uint32_t>(a_flag));

    this->transmit_start(a_priority, a_mode, a_p_buffer, a_buffer_size_in_words);
}

void DMA<RS485>::transmit_stop()
{
    bit_flag::clear(&(this->p_RS485_registers->CR3), USART_CR3_DMAT);
    bit_flag::clear(&(this->p_request_registers->CSELR), this->tx_cselr.clear);

    this->p_tx_channel_registers->CCR   = 0x0u;
    this->p_tx_channel_registers->CNDTR = 0x0u;
    this->p_tx_channel_registers->CPAR  = 0x0u;
    this->p_tx_channel_registers->CMAR  = 0x0u;

    NVIC_DisableIRQ(this->tx_irqn);

    if (nullptr != this->tx_callback.function)
    {
        this->clear_irq_context();
    }

    this->tx_callback = { nullptr, nullptr };
}

void DMA<RS485>::receive_start(DMA<>::Priority a_priority,
                               DMA<>::Mode a_mode,
                               void* a_p_buffer,
                               std::uint16_t a_buffer_size_in_words)
{
    receive(this->p_RS485_registers,
            this->p_request_registers,
            this->p_rx_channel_registers,
            static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) |
                (USART_CR1_M0 == bit_flag::get(this->p_RS485_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                     (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                     0x0u),
            this->rx_cselr,
            a_p_buffer,
            a_buffer_size_in_words);
}

void DMA<RS485>::receive_start(DMA<>::Priority a_priority,
                               DMA<>::Mode a_mode,
                               void* a_p_buffer,
                               std::uint16_t a_buffer_size_in_words,
                               const IRQ_config& a_irq_config,
                               const DMA<>::Callback& a_callback,
                               DMA<>::Event_flag a_flag)
{
    cml_assert(nullptr != a_callback.function);

    this->rx_callback = a_callback;

    this->set_rx_irq_context();

    NVIC_SetPriority(
        this->rx_irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->rx_irqn);

    bit_flag::set(&(this->p_rx_channel_registers->CCR), static_cast<std::uint32_t>(a_flag));

    this->receive_start(a_priority, a_mode, a_p_buffer, a_buffer_size_in_words);
}

void DMA<RS485>::receive_stop()
{
    bit_flag::clear(&(this->p_RS485_registers->CR3), USART_CR3_DMAR);
    bit_flag::clear(&(this->p_request_registers->CSELR), this->rx_cselr.clear);

    this->p_rx_channel_registers->CCR   = 0x0u;
    this->p_rx_channel_registers->CNDTR = 0x0u;
    this->p_rx_channel_registers->CPAR  = 0x0u;
    this->p_rx_channel_registers->CMAR  = 0x0u;

    NVIC_DisableIRQ(this->rx_irqn);

    if (nullptr != this->rx_callback.function)
    {
        this->clear_irq_context();
    }

    this->rx_callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif