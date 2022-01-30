/*
 *   Name: DMA.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// soc
#include <soc/m4/stm32l4/USART/DMA.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {
using namespace cml;
using namespace soc::m4::stm32l4;

DMA<>::Callback* p_callbacks[2][7] = { nullptr };

DMA<>::Event_flag get_Event_flag_and_clear(DMA_TypeDef* a_p_DMA_registers,
                                           DMA_Channel_TypeDef* a_p_channel_registers,
                                           std::size_t a_channel_idx)
{
    DMA<>::Event_flag ret   = DMA<>::Event_flag::none;
    const std::uint32_t isr = a_p_DMA_registers->ISR;

    if (true == bit::is(isr, DMA_ISR_TCIF1_Pos + a_channel_idx))
    {
        bit::set(&(a_p_DMA_registers->IFCR), DMA_IFCR_CTCIF1_Pos + a_channel_idx);

        if (true == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_TCIE))
        {
            ret |= DMA<>::Event_flag::transfer_complete;
        }
    }

    if (true == bit::is(isr, DMA_ISR_HTIF1_Pos + a_channel_idx))
    {
        bit::set(&(a_p_DMA_registers->IFCR), DMA_IFCR_CHTIF1_Pos + a_channel_idx);

        if (true == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_HTIE))
        {
            ret |= DMA<>::Event_flag::half_transfer;
        }
    }

    if (true == bit::is(isr, DMA_ISR_TEIF1_Pos + a_channel_idx))
    {
        bit::set(&(a_p_DMA_registers->IFCR), DMA_IFCR_CTEIF1_Pos + a_channel_idx);

        if (true == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_TEIE))
        {
            ret |= DMA<>::Event_flag::transfer_error;
        }
    }

    return ret;
}

} // namespace

extern "C" {
void DMA1_Channel1_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[0][0]);

    p_callbacks[0][0]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, 0), p_callbacks[0][0]->p_user_data);
}
void DMA1_Channel2_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[0][1]);

    p_callbacks[0][1]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel2, 1), p_callbacks[0][1]->p_user_data);
}
void DMA1_Channel3_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[0][2]);

    p_callbacks[0][2]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel3, 2), p_callbacks[0][2]->p_user_data);
}
void DMA1_Channel4_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[0][3]);

    p_callbacks[0][3]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel4, 3), p_callbacks[0][3]->p_user_data);
}
void DMA1_Channel5_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[0][4]);

    p_callbacks[0][4]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel5, 4), p_callbacks[0][4]->p_user_data);
}
void DMA1_Channel6_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[0][5]);

    p_callbacks[0][5]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel6, 5), p_callbacks[0][5]->p_user_data);
}
void DMA1_Channel7_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[0][6]);

    p_callbacks[0][6]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel7, 6), p_callbacks[0][6]->p_user_data);
}

void DMA2_Channel1_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[1][0]);

    p_callbacks[1][0]->function(get_Event_flag_and_clear(DMA2, DMA2_Channel1, 0), p_callbacks[1][0]->p_user_data);
}
void DMA2_Channel2_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[0][1]);

    p_callbacks[1][1]->function(get_Event_flag_and_clear(DMA2, DMA2_Channel2, 1), p_callbacks[1][1]->p_user_data);
}
void DMA2_Channel3_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[1][2]);

    p_callbacks[1][2]->function(get_Event_flag_and_clear(DMA2, DMA2_Channel3, 2), p_callbacks[1][2]->p_user_data);
}
void DMA2_Channel4_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[1][3]);

    p_callbacks[1][3]->function(get_Event_flag_and_clear(DMA2, DMA2_Channel4, 3), p_callbacks[1][3]->p_user_data);
}
void DMA2_Channel5_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[1][4]);

    p_callbacks[1][4]->function(get_Event_flag_and_clear(DMA2, DMA2_Channel5, 4), p_callbacks[1][4]->p_user_data);
}
void DMA2_Channel6_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[1][5]);

    p_callbacks[1][5]->function(get_Event_flag_and_clear(DMA2, DMA2_Channel6, 5), p_callbacks[1][5]->p_user_data);
}
void DMA2_Channel7_IRQHandler()
{
    cml_assert(nullptr != p_callbacks[1][6]);

    p_callbacks[1][6]->function(get_Event_flag_and_clear(DMA2, DMA2_Channel7, 6), p_callbacks[1][6]->p_user_data);
}
}

namespace soc {
namespace m4 {
namespace stm32l4 {
void DMA<USART>::set_tx_irq_context()
{
    cml_assert(nullptr == p_callbacks[this->idx][this->tx_irqn - DMA1_Channel1_IRQn]);

    p_callbacks[this->idx][this->tx_irqn - DMA1_Channel1_IRQn] = &(this->tx_callback);
}

void DMA<USART>::set_rx_irq_context()
{
    cml_assert(nullptr == p_callbacks[this->idx][this->tx_irqn - DMA1_Channel1_IRQn]);

    p_callbacks[this->idx][this->rx_irqn - DMA1_Channel1_IRQn] = &(this->rx_callback);
}

void DMA<USART>::clear_irq_context()
{
    cml_assert(nullptr != p_callbacks[this->idx][this->tx_irqn - DMA1_Channel1_IRQn]);

    p_callbacks[this->idx][this->tx_irqn - DMA1_Channel1_IRQn] = nullptr;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif