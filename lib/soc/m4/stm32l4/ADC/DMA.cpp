/*
 *   Name: DMA.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/ADC/DMA.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {
using namespace cml;
using namespace soc::m4::stm32l4;

void start(ADC_TypeDef* a_p_ADC_registers,
           DMA_Request_TypeDef* a_p_request_registers,
           DMA_Channel_TypeDef* a_p_channel_registers,
           std::uint32_t a_channel_flags,
           const DMA<>::CSELR& a_cselr,
           ADC::Mode a_mode,
           void* a_p_buffer,
           std::uint16_t a_buffer_size)
{
    cml_assert(false == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_EN));

    a_p_channel_registers->CNDTR = a_buffer_size;
    a_p_channel_registers->CPAR  = reinterpret_cast<std::uint32_t>(&(a_p_ADC_registers->DR));
    a_p_channel_registers->CMAR  = reinterpret_cast<std::uint32_t>(a_p_buffer);

    bit_flag::set(&(a_p_ADC_registers->CFGR),
                  ADC_CFGR_CONT | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_Msk,
                  static_cast<uint32_t>(a_mode));

    bit_flag::set(&(a_p_ADC_registers->CFGR),
                  ADC_CFGR_DMAEN | (true == bit_flag::is(a_channel_flags, DMA_CCR_CIRC) ? ADC_CFGR_DMACFG : 0x0u));

    bit_flag::set(&(a_p_request_registers->CSELR), a_cselr.clear, a_cselr.set);
    bit_flag::set(&(a_p_channel_registers->CCR), a_channel_flags | DMA_CCR_EN);

    bit_flag::set(&(a_p_ADC_registers->CR), ADC_CR_ADSTART);
}
} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

template<> void DMA<ADC>::read_start<ADC::Mode::single>(DMA<>::Priority a_priority,
                                                        DMA<>::Mode a_mode,
                                                        void* a_p_buffer,
                                                        std::uint16_t a_buffer_size)
{
    start(this->p_ADC_registers,
          this->p_request_registers,
          this->p_channel_registers,
          static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
              (a_buffer_size > 1 ? DMA_CCR_MINC : 0x0u) |
              (false == bit::is(this->p_ADC_registers->CFGR, ADC_CFGR_RES_1) ? (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                                                                               0x0u),
          this->cselr,
          ADC::Mode::single,
          a_p_buffer,
          a_buffer_size);
}

template<> void DMA<ADC>::read_start<ADC::Mode::continuous>(DMA<>::Priority a_priority,
                                                            DMA<>::Mode a_mode,
                                                            void* a_p_buffer,
                                                            std::uint16_t a_buffer_size)
{
    start(this->p_ADC_registers,
          this->p_request_registers,
          this->p_channel_registers,
          static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
              (a_buffer_size > 1 ? DMA_CCR_MINC : 0x0u) |
              (false == bit::is(this->p_ADC_registers->CFGR, ADC_CFGR_RES_1) ? (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                                                                               0x0u),
          this->cselr,
          ADC::Mode::continuous,
          a_p_buffer,
          a_buffer_size);
}

template<> void DMA<ADC>::read_start<ADC::Mode::single>(DMA<>::Priority a_priority,
                                                        DMA<>::Mode a_mode,
                                                        void* a_p_buffer,
                                                        std::uint16_t a_buffer_size,
                                                        const IRQ_config& a_irq_config,
                                                        const DMA<>::Callback& a_callback,
                                                        DMA<>::Event_flag a_flag)
{
    cml_assert(nullptr != a_callback.function);

    this->callback = a_callback;

    this->set_irq_context();

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    bit_flag::set(&(this->p_channel_registers->CCR), static_cast<std::uint32_t>(a_flag));

    this->read_start<ADC::Mode::single>(a_priority, a_mode, a_p_buffer, a_buffer_size);
}

template<> void DMA<ADC>::read_start<ADC::Mode::continuous>(DMA<>::Priority a_priority,
                                                            DMA<>::Mode a_mode,
                                                            void* a_p_buffer,
                                                            std::uint16_t a_buffer_size,
                                                            const IRQ_config& a_irq_config,
                                                            const DMA<>::Callback& a_callback,
                                                            DMA<>::Event_flag a_flag)
{
    cml_assert(nullptr != a_callback.function);

    this->callback = a_callback;

    this->set_irq_context();

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    bit_flag::set(&(this->p_channel_registers->CCR), static_cast<std::uint32_t>(a_flag));

    this->read_start<ADC::Mode::continuous>(a_priority, a_mode, a_p_buffer, a_buffer_size);
}

template<> void DMA<ADC>::read_start<ADC::Mode::discontinuous>(DMA<>::Priority a_priority,
                                                               DMA<>::Mode a_DMA_mode,
                                                               void* a_p_buffer,
                                                               std::uint16_t a_buffer_size,
                                                               std::uint16_t a_group_size)
{
}

template<> void DMA<ADC>::read_start<ADC::Mode::discontinuous>(DMA<>::Priority a_priority,
                                                               DMA<>::Mode a_DMA_mode,
                                                               void* a_p_buffer,
                                                               std::uint16_t a_buffer_size,
                                                               std::uint16_t a_group_size,
                                                               const IRQ_config& a_irq_config,
                                                               const DMA<>::Callback& a_callback,
                                                               DMA<>::Event_flag a_flag)
{
}

void DMA<ADC>::read_stop()
{
    bit_flag::clear(&(this->p_request_registers->CSELR), this->cselr.clear);

    this->p_channel_registers->CCR   = 0x0u;
    this->p_channel_registers->CNDTR = 0x0u;
    this->p_channel_registers->CPAR  = 0x0u;
    this->p_channel_registers->CMAR  = 0x0u;

    NVIC_DisableIRQ(this->irqn);

    if (nullptr != this->callback.function)
    {
        this->clear_irq_context();
    }

    this->callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif