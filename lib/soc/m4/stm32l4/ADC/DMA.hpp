#pragma once

/*
 *   Name: DMA.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/ADC/ADC.hpp>
#include <soc/m4/stm32l4/DMA.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class DMA<ADC> : private cml::Non_copyable
{
public:
    DMA(DMA&&)   = default;
    DMA& operator=(DMA&&) = default;

    DMA()
        : idx(std::numeric_limits<std::size_t>::max())
        , p_ADC_registers(nullptr)
        , p_request_registers(nullptr)
        , p_channel_registers(nullptr)
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
    }

    template<ADC::Mode mode> void
    read_start(DMA<>::Priority a_priority, DMA<>::Mode a_mode, void* a_p_buffer, std::uint16_t a_buffer_size) = delete;

    template<ADC::Mode mode> void read_start(DMA<>::Priority a_priority,
                                             DMA<>::Mode a_mode,
                                             void* a_p_buffer,
                                             std::uint16_t a_buffer_size,
                                             std::uint16_t a_group_size) = delete;

    template<ADC::Mode mode> void read_start(DMA<>::Priority a_priority,
                                             DMA<>::Mode a_mode,
                                             void* a_p_buffer,
                                             std::uint16_t a_buffer_size,
                                             const IRQ_config& a_irq_config,
                                             const DMA<>::Callback& a_callback,
                                             DMA<>::Event_flag a_flag) = delete;

    template<ADC::Mode mode> void read_start(DMA<>::Priority a_priority,
                                             DMA<>::Mode a_mode,
                                             void* a_p_buffer,
                                             std::uint16_t a_buffer_size,
                                             std::uint16_t a_group_size,
                                             const IRQ_config& a_irq_config,
                                             const DMA<>::Callback& a_callback,
                                             DMA<>::Event_flag a_flag) = delete;

    void read_stop();

    std::size_t get_idx() const
    {
        return this->idx;
    }

private:
    DMA(std::size_t a_idx,
        ADC_TypeDef* a_p_ADC_registers,
        DMA_Request_TypeDef* a_p_request_registers,
        DMA_Channel_TypeDef* a_p_channel_registers,
        IRQn_Type a_irqn,
        const DMA<>::CSELR& a_cselr)
        : idx(a_idx)
        , p_ADC_registers(a_p_ADC_registers)
        , p_request_registers(a_p_request_registers)
        , p_channel_registers(a_p_channel_registers)
        , irqn(a_irqn)
        , cselr(a_cselr)
    {
    }

    void set_irq_context();
    void clear_irq_context();

    std::size_t idx;

    ADC_TypeDef* p_ADC_registers;
    DMA_Request_TypeDef* p_request_registers;

    DMA_Channel_TypeDef* p_channel_registers;
    IRQn_Type irqn;
    DMA<>::CSELR cselr;

    DMA<>::Callback callback;

    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;
};

template<> void DMA<ADC>::read_start<ADC::Mode::single>(DMA<>::Priority a_priority,
                                                        DMA<>::Mode a_mode,
                                                        void* a_p_buffer,
                                                        std::uint16_t a_buffer_size);

template<> void DMA<ADC>::read_start<ADC::Mode::continuous>(DMA<>::Priority a_priority,
                                                            DMA<>::Mode a_mode,
                                                            void* a_p_buffer,
                                                            std::uint16_t a_buffer_size);

template<> void DMA<ADC>::read_start<ADC::Mode::single>(DMA<>::Priority a_priority,
                                                        DMA<>::Mode a_mode,
                                                        void* a_p_buffer,
                                                        std::uint16_t a_buffer_size,
                                                        const IRQ_config& a_irq_config,
                                                        const DMA<>::Callback& a_callback,
                                                        DMA<>::Event_flag a_flag);

template<> void DMA<ADC>::read_start<ADC::Mode::continuous>(DMA<>::Priority a_priority,
                                                            DMA<>::Mode a_mode,
                                                            void* a_p_buffer,
                                                            std::uint16_t a_buffer_size,
                                                            const IRQ_config& a_irq_config,
                                                            const DMA<>::Callback& a_callback,
                                                            DMA<>::Event_flag a_flag);

template<> void DMA<ADC>::read_start<ADC::Mode::discontinuous>(DMA<>::Priority a_priority,
                                                               DMA<>::Mode a_mode,
                                                               void* a_p_buffer,
                                                               std::uint16_t a_buffer_size,
                                                               std::uint16_t a_group_size);

template<> void DMA<ADC>::read_start<ADC::Mode::discontinuous>(DMA<>::Priority a_priority,
                                                               DMA<>::Mode a_mode,
                                                               void* a_p_buffer,
                                                               std::uint16_t a_buffer_size,
                                                               std::uint16_t a_group_size,
                                                               const IRQ_config& a_irq_config,
                                                               const DMA<>::Callback& a_callback,
                                                               DMA<>::Event_flag a_flag);
} // namespace stm32l4
} // namespace m4
} // namespace soc