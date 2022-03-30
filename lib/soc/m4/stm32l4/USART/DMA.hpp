#pragma once

/*
 *   Name: DMA.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <limits>

// soc
#include <soc/m4/IRQ_config.hpp>
#include <soc/m4/stm32l4/DMA.hpp>
#include <soc/m4/stm32l4/USART/RS485.hpp>
#include <soc/m4/stm32l4/USART/USART.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class DMA<USART> : private cml::Non_copyable
{
public:
    DMA(DMA&&)   = default;
    DMA& operator=(DMA&&) = default;

    DMA()
        : idx(std::numeric_limits<std::size_t>::max())
        , p_USART_registers(nullptr)
        , p_request_registers(nullptr)
        , p_tx_channel_registers(nullptr)
        , tx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
        , p_rx_channel_registers(nullptr)
        , rx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
    }

    void transmit_start(DMA<>::Priority a_priority,
                        DMA<>::Mode a_mode,
                        const void* a_p_buffer,
                        std::uint16_t a_buffer_size_in_words);
    void transmit_start(DMA<>::Priority a_priority,
                        DMA<>::Mode a_mode,
                        const void* a_p_buffer,
                        std::uint16_t a_buffer_size_in_words,
                        const IRQ_config& a_irq_config,
                        const DMA<>::Callback& a_callback,
                        DMA<>::Event_flag a_flag);
    void transmit_stop();

    void receive_start(DMA<>::Priority a_priority,
                       DMA<>::Mode a_mode,
                       void* a_p_buffer,
                       std::uint16_t a_buffer_size_in_words);
    void receive_start(DMA<>::Priority a_priority,
                       DMA<>::Mode a_mode,
                       void* a_p_buffer,
                       std::uint16_t a_buffer_size_in_words,
                       const IRQ_config& a_irq_config,
                       const DMA<>::Callback& a_callback,
                       DMA<>::Event_flag a_flag);
    void receive_stop();

    std::size_t get_idx() const
    {
        return this->idx;
    }

private:
    DMA(std::size_t a_idx,
        USART_TypeDef* a_p_USART_registers,
        DMA_Request_TypeDef* a_p_request_registers,
        DMA_Channel_TypeDef* a_p_tx_channel_registers,
        IRQn_Type a_tx_irqn,
        const DMA<>::CSELR& a_tx_cselr,
        DMA_Channel_TypeDef* a_p_rx_channel_registers,
        IRQn_Type a_rx_irqn,
        const DMA<>::CSELR& a_rx_cselr)
        : idx(a_idx)
        , p_USART_registers(a_p_USART_registers)
        , p_request_registers(a_p_request_registers)

        , p_tx_channel_registers(a_p_tx_channel_registers)
        , tx_irqn(a_tx_irqn)
        , tx_cselr(a_tx_cselr)

        , p_rx_channel_registers(a_p_rx_channel_registers)
        , rx_irqn(a_rx_irqn)
        , rx_cselr(a_rx_cselr)
    {
    }

    void set_tx_irq_context();
    void set_rx_irq_context();
    void clear_irq_context();

    std::size_t idx;

    USART_TypeDef* p_USART_registers;
    DMA_Request_TypeDef* p_request_registers;

    DMA_Channel_TypeDef* p_tx_channel_registers;
    IRQn_Type tx_irqn;
    DMA<>::CSELR tx_cselr;

    DMA_Channel_TypeDef* p_rx_channel_registers;
    IRQn_Type rx_irqn;
    DMA<>::CSELR rx_cselr;

    DMA<>::Callback tx_callback;
    DMA<>::Callback rx_callback;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

template<> class DMA<RS485> : private cml::Non_copyable
{
    DMA(DMA&&)   = default;
    DMA& operator=(DMA&&) = default;

    DMA()
        : idx(std::numeric_limits<std::size_t>::max())
        , p_RS485_registers(nullptr)
        , p_request_registers(nullptr)
        , p_tx_channel_registers(nullptr)
        , tx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
        , p_rx_channel_registers(nullptr)
        , rx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
    }

    void transmit_start(DMA<>::Priority a_priority,
                        DMA<>::Mode a_mode,
                        const void* a_p_buffer,
                        std::uint16_t a_buffer_size_in_words);
    void transmit_start(DMA<>::Priority a_priority,
                        DMA<>::Mode a_mode,
                        const void* a_p_buffer,
                        std::uint16_t a_buffer_size_in_words,
                        const IRQ_config& a_irq_config,
                        const DMA<>::Callback& a_callback,
                        DMA<>::Event_flag a_flag);
    void transmit_stop();

    void receive_start(DMA<>::Priority a_priority,
                       DMA<>::Mode a_mode,
                       void* a_p_buffer,
                       std::uint16_t a_buffer_size_in_words);
    void receive_start(DMA<>::Priority a_priority,
                       DMA<>::Mode a_mode,
                       void* a_p_buffer,
                       std::uint16_t a_buffer_size_in_words,
                       const IRQ_config& a_irq_config,
                       const DMA<>::Callback& a_callback,
                       DMA<>::Event_flag a_flag);
    void receive_stop();

    std::size_t get_idx()
    {
        return this->idx;
    }

private:
    DMA(std::size_t a_idx,
        USART_TypeDef* a_p_RS485_registers,
        DMA_Request_TypeDef* a_p_request_registers,
        DMA_Channel_TypeDef* a_p_tx_channel_registers,
        IRQn_Type a_tx_irqn,
        const DMA<>::CSELR& a_tx_cselr,
        DMA_Channel_TypeDef* a_p_rx_channel_registers,
        IRQn_Type a_rx_irqn,
        const DMA<>::CSELR& a_rx_cselr)
        : idx(a_idx)
        , p_RS485_registers(a_p_RS485_registers)
        , p_request_registers(a_p_request_registers)

        , p_tx_channel_registers(a_p_tx_channel_registers)
        , tx_irqn(a_tx_irqn)
        , tx_cselr(a_tx_cselr)

        , p_rx_channel_registers(a_p_rx_channel_registers)
        , rx_irqn(a_rx_irqn)
        , rx_cselr(a_rx_cselr)
    {
    }

    void set_tx_irq_context();
    void set_rx_irq_context();
    void clear_irq_context();

    std::size_t idx;

    USART_TypeDef* p_RS485_registers;
    DMA_Request_TypeDef* p_request_registers;

    DMA_Channel_TypeDef* p_tx_channel_registers;
    IRQn_Type tx_irqn;
    DMA<>::CSELR tx_cselr;

    DMA_Channel_TypeDef* p_rx_channel_registers;
    IRQn_Type rx_irqn;
    DMA<>::CSELR rx_cselr;

    DMA<>::Callback tx_callback;
    DMA<>::Callback rx_callback;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc