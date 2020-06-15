/*
    Name: USART.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <hal/stm32l452xx/USART.hpp>

//cml
#include <debug/assert.hpp>
#include <hal/core/system_counter.hpp>
#include <utils/wait.hpp>

namespace {

using namespace cml::common;
using namespace cml::hal::core;
using namespace cml::hal::stm32l452xx;

void usart_1_enable(USART::Clock::Source a_clock_source, uint32 a_irq_priority)
{
    assert(a_clock_source != USART::Clock::Source::unknown);

    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART1SEL_0, RCC_CCIPR_USART1SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART1SEL, clock_source_lut[static_cast<uint32>(a_clock_source)]);
    set_flag(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);

    NVIC_SetPriority(USART1_IRQn, a_irq_priority);
    NVIC_EnableIRQ(USART1_IRQn);
}

void usart_1_disable()
{
    clear_flag(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);
    NVIC_DisableIRQ(USART1_IRQn);
}

void usart_2_enable(USART::Clock::Source a_clock_source, uint32 a_irq_priority)
{
    assert(a_clock_source != USART::Clock::Source::unknown);

    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART2SEL_0, RCC_CCIPR_USART2SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART2SEL, clock_source_lut[static_cast<uint32>(a_clock_source)]);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN);

    NVIC_SetPriority(USART2_IRQn, a_irq_priority);
    NVIC_EnableIRQ(USART2_IRQn);
}

void usart_2_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN);
    NVIC_DisableIRQ(USART2_IRQn);
}

void usart_3_enable(USART::Clock::Source a_clock_source, uint32 a_irq_priority)
{
    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART3SEL_0, RCC_CCIPR_USART3SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART3SEL, clock_source_lut[static_cast<int32>(a_clock_source)]);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN);

    NVIC_SetPriority(USART3_IRQn, a_irq_priority);
    NVIC_EnableIRQ(USART3_IRQn);
}

void usart_3_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN);
    NVIC_DisableIRQ(USART3_IRQn);
}

bool is_USART_ISR_error(uint32 a_isr)
{
    return is_any_bit(a_isr, USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
}

USART::Bus_status_flag get_bus_status_flag_from_USART_ISR(uint32 a_isr)
{
    USART::Bus_status_flag ret = USART::Bus_status_flag::ok;

    if (true == is_flag(a_isr, USART_ISR_PE))
    {
        ret |= USART::Bus_status_flag::parity_error;
    }

    if (true == is_flag(a_isr, USART_ISR_FE))
    {
        ret |= USART::Bus_status_flag::framing_error;
    }

    if (true == is_flag(a_isr, USART_ISR_ORE))
    {
        ret |= USART::Bus_status_flag::overrun;
    }

    if (true == is_flag(a_isr, USART_ISR_NE))
    {
        ret |= USART::Bus_status_flag::noise_detected;
    }

    return ret;
}

void clear_USART_ISR_errors(volatile uint32* a_p_icr)
{
    set_flag(a_p_icr, USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
}

struct Controller
{
    using Enable_function  = void(*)(USART::Clock::Source a_clock_source, uint32 a_irq_priority);
    using Disable_function = void(*)();

    USART_TypeDef* p_registers = nullptr;
    USART* p_usart_handle      = nullptr;

    Enable_function enable   = nullptr;
    Disable_function disable = nullptr;
};

Controller controllers[] =
{
    { USART1, nullptr, usart_1_enable, usart_1_disable },
    { USART2, nullptr, usart_2_enable, usart_2_disable },
    { USART3, nullptr, usart_3_enable, usart_3_disable }
};

} // namespace ::

extern "C"
{

void USART1_IRQHandler()
{
    assert(nullptr != controllers[0].p_usart_handle);
    usart_interrupt_handler(controllers[0].p_usart_handle);
}

void USART2_IRQHandler()
{
    assert(nullptr != controllers[1].p_usart_handle);
    usart_interrupt_handler(controllers[1].p_usart_handle);
}

void USART3_IRQHandler()
{
    assert(nullptr != controllers[2].p_usart_handle);
    usart_interrupt_handler(controllers[2].p_usart_handle);
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;
using namespace cml::hal::core;
using namespace cml::utils;

void usart_interrupt_handler(USART* a_p_this)
{
    assert(nullptr != a_p_this);

    uint32 isr = a_p_this->p_usart->ISR;
    uint32 cr1 = a_p_this->p_usart->CR1;
    uint32 cr3 = a_p_this->p_usart->CR3;

    if (true == is_flag(isr, USART_ISR_TXE) &&
        true == is_flag(cr1, USART_CR1_TXEIE) &&
        nullptr != a_p_this->tx_callback.function)
    {
        if (false == a_p_this->tx_callback.function(&(a_p_this->p_usart->TDR), false, a_p_this->tx_callback.p_user_data))
        {
            a_p_this->unregister_transmit_callback();
        }
    }

    if (true == is_flag(isr, USART_ISR_TC) &&
        true == is_flag(cr1, USART_CR1_TCIE) &&
        nullptr != a_p_this->tx_callback.function)
    {
        if (false == a_p_this->tx_callback.function(nullptr, true, a_p_this->tx_callback.p_user_data))
        {
            a_p_this->unregister_transmit_callback();
        }
    }

    if (nullptr != a_p_this->rx_callback.function)
    {
        bool status = true;

        if (true == is_flag(isr, USART_ISR_RXNE) && true == is_flag(cr1, USART_CR1_RXNEIE))
        {
            status = a_p_this->rx_callback.function(a_p_this->p_usart->RDR,
                                                    false,
                                                    a_p_this->rx_callback.p_user_data);
        }
        
        else if (true == is_flag(isr, USART_ISR_IDLE) && true == is_flag(cr1, USART_CR1_IDLEIE))
        {
            set_flag(&(a_p_this->p_usart->ICR), USART_ICR_IDLECF);
            status = a_p_this->rx_callback.function(0x0u, true, a_p_this->rx_callback.p_user_data);
        }

        if (false == status)
        {
            a_p_this->unregister_receive_callback();
        }
    }

    if (nullptr != a_p_this->bus_status_callback.function && true == is_flag(cr3, USART_CR3_EIE | USART_CR1_PEIE))
    {
        USART::Bus_status_flag status = get_bus_status_flag_from_USART_ISR(isr);

        if (status != USART::Bus_status_flag::ok &&
            true == a_p_this->bus_status_callback.function(status, a_p_this->bus_status_callback.p_user_data))
        {
            clear_USART_ISR_errors(&(a_p_this->p_usart->ICR));
        }
    }
}

bool USART::enable(const Config& a_config, const Clock &a_clock, uint32 a_irqn_priority, time::tick a_timeout)
{
    assert(nullptr               == this->p_usart);
    assert(a_config.baud_rate    != 0);
    assert(a_config.flow_control != Flow_control::unknown);
    assert(a_config.parity       != Parity::unknown);
    assert(a_config.stop_bits    != Stop_bits::unknown);
    assert(a_config.word_length  != Word_length::unknown);

    assert(Clock::Source::unknown != a_clock.source);
    assert(0                      != a_clock.frequency_hz);
    assert(a_timeout > 0);

    time::tick start = system_counter::get();

    controllers[static_cast<uint32>(this->id)].p_usart_handle = this;
    this->p_usart = controllers[static_cast<uint32>(this->id)].p_registers;

    controllers[static_cast<uint32>(this->id)].enable(a_clock.source, a_irqn_priority);

    this->p_usart->CR1 = 0;
    this->p_usart->CR2 = 0;
    this->p_usart->CR3 = 0;

    switch (a_config.oversampling)
    {
        case Oversampling::_16:
        {
            this->p_usart->BRR = a_clock.frequency_hz / a_config.baud_rate;
        }
        break;

        case Oversampling::_8:
        {
            uint32 usartdiv = 2 * a_clock.frequency_hz / a_config.baud_rate;
            this->p_usart->BRR = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::unknown:
        {
            assert(a_config.oversampling != Oversampling::unknown);
        }
        break;
    }

    this->p_usart->CR2 = static_cast<uint32>(a_config.stop_bits);
    this->p_usart->CR3 = static_cast<uint32>(a_config.flow_control);

    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_config.word_length)  |
                                    static_cast<uint32>(a_config.oversampling) |
                                    static_cast<uint32>(a_config.parity));

    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), USART_CR1_TE | USART_CR1_RE);

    this->baud_rate = a_config.baud_rate;
    this->clock     = a_clock;

    bool ret = wait::until(&(this->p_usart->ISR), USART_ISR_TEACK | USART_ISR_REACK, false, start, a_timeout);

    if (false == ret)
    {
        this->disable();
    }

    return ret;
}

void USART::disable()
{
    assert(nullptr != this->p_usart);

    this->p_usart->CR1 = 0;
    this->p_usart->CR2 = 0;
    this->p_usart->CR3 = 0;

    controllers[static_cast<uint32>(this->id)].disable();
    controllers[static_cast<uint32>(this->id)].p_usart_handle = nullptr;

    this->p_usart = nullptr;
}

uint32 USART::transmit_bytes_polling(const void* a_p_data, uint32 a_data_size_in_bytes, Bus_status_flag* a_p_status)
{
    assert(nullptr != this->p_usart);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);

    set_flag(&(USART2->ICR), USART_ICR_TCCF);

    uint32 ret  = 0;
    while (false == is_flag(USART2->ISR, USART_ISR_TC) && false == is_USART_ISR_error(this->p_usart->ISR))
    {
        if (true == is_flag(this->p_usart->ISR, USART_ISR_TXE) && ret < a_data_size_in_bytes)
        {
            this->p_usart->TDR = static_cast<const uint8*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_flag_from_USART_ISR(this->p_usart->ISR);
    }

    if (true == is_USART_ISR_error(this->p_usart->ISR))
    {
        clear_USART_ISR_errors(&(this->p_usart->ICR));
    }

    return ret;
}

uint32 USART::transmit_bytes_polling(const void* a_p_data,
                                     uint32 a_data_size_in_bytes,
                                     time::tick a_timeout,
                                     Bus_status_flag* a_p_status)
{
    assert(nullptr != this->p_usart);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);
    assert(a_timeout > 0);

    time::tick start = system_counter::get();

    set_flag(&(USART2->ICR), USART_ICR_TCCF);

    uint32 ret = 0;
    while (false == is_flag(USART2->ISR, USART_ISR_TC) &&
           false == is_USART_ISR_error(this->p_usart->ISR) &&
           a_timeout < time::diff(system_counter::get(), start))
    {
        if (true == is_flag(this->p_usart->ISR, USART_ISR_TXE) && ret < a_data_size_in_bytes)
        {   
            this->p_usart->TDR = static_cast<const uint8*>(a_p_data)[ret++];
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_flag_from_USART_ISR(this->p_usart->ISR);
    }

    if (true == is_USART_ISR_error(this->p_usart->ISR))
    {
        clear_USART_ISR_errors(&(this->p_usart->ICR));
    }

    return ret;
}

uint32 USART::receive_bytes_polling(void* a_p_data, uint32 a_data_size_in_bytes, Bus_status_flag* a_p_status)
{
    assert(nullptr != this->p_usart);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);

    set_flag(&(this->p_usart->ICR), USART_ICR_IDLECF);

    uint32 ret = 0;

    while ((ret < a_data_size_in_bytes || false == is_flag(this->p_usart->ISR, USART_ISR_IDLE)) &&
           false == is_USART_ISR_error(this->p_usart->ISR))
    {
        if (true == is_flag(this->p_usart->ISR, USART_ISR_RXNE))
        {
            if (ret < a_data_size_in_bytes)
            {
                static_cast<uint8*>(a_p_data)[ret++] = this->p_usart->RDR;
            }
            else
            {
                set_flag(&(this->p_usart->RQR), USART_RQR_RXFRQ);
                ret++;
            }
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_flag_from_USART_ISR(this->p_usart->ISR);
    }

    if (true == is_USART_ISR_error(this->p_usart->ISR))
    {
        clear_USART_ISR_errors(&(this->p_usart->ICR));
    }

    return ret;
}

uint32 USART::receive_bytes_polling(void* a_p_data,
                                    uint32 a_data_size_in_bytes,
                                    time::tick a_timeout,
                                    Bus_status_flag* a_p_status)
{
    assert(nullptr != this->p_usart);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);
    assert(a_timeout > 0);

    time::tick start = system_counter::get();

    set_flag(&(this->p_usart->ICR), USART_ICR_IDLECF);

    uint32 ret = 0;

    while ((ret < a_data_size_in_bytes || false == is_flag(this->p_usart->ISR, USART_ISR_IDLE)) &&
           false == is_USART_ISR_error(this->p_usart->ISR) &&
           a_timeout < time::diff(system_counter::get(), start))
    {
        if (true == is_flag(this->p_usart->ISR, USART_ISR_RXNE))
        {
            if (ret < a_data_size_in_bytes)
            {
                static_cast<uint8*>(a_p_data)[ret++] = this->p_usart->RDR;
            }
            else
            {
                set_flag(&(this->p_usart->RQR), USART_RQR_RXFRQ);
                ret++;
            }
        }
    }

    if (nullptr != a_p_status)
    {
        (*a_p_status) = get_bus_status_flag_from_USART_ISR(this->p_usart->ISR);
    }

    if (true == is_USART_ISR_error(this->p_usart->ISR))
    {
        clear_USART_ISR_errors(&(this->p_usart->ICR));
    }

    return ret;
}

void USART::register_transmit_callback(const TX_callback& a_callback)
{
    assert(nullptr != this->p_usart);
    assert(nullptr != a_callback.function);

    this->tx_callback = a_callback;

    set_flag(&(this->p_usart->ICR), USART_ICR_TCCF);
    set_flag(&(this->p_usart->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);
}

void USART::register_receive_callback(const RX_callback& a_callback)
{
    assert(nullptr != this->p_usart);
    assert(nullptr != a_callback.function);

    this->rx_callback = a_callback;

    set_flag(&(this->p_usart->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
}

void USART::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    assert(nullptr != this->p_usart);
    assert(nullptr != a_callback.function);

    this->bus_status_callback = a_callback;

    set_flag(&(this->p_usart->CR3), USART_CR3_EIE | USART_CR1_PEIE);
}

void USART::unregister_transmit_callback()
{
    assert(nullptr != this->p_usart);

    clear_flag(&(this->p_usart->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);

    this->tx_callback = { nullptr, nullptr };
}

void USART::unregister_receive_callback()
{
    assert(nullptr != this->p_usart);

    clear_flag(&(this->p_usart->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);

    this->rx_callback = { nullptr, nullptr };
}

void USART::unregister_bus_status_callback()
{
    clear_flag(&(this->p_usart->CR3), USART_CR3_EIE | USART_CR1_PEIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void USART::set_baud_rate(uint32 a_baud_rate)
{
    assert(nullptr != this->p_usart);
    assert(0 != a_baud_rate);

    const Oversampling oversampling = this->get_oversampling();

    switch (oversampling)
    {
        case Oversampling::_8:
        {
            uint32 usartdiv = 2 * this->clock.frequency_hz / a_baud_rate;
            this->p_usart->BRR = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::_16:
        {
            this->p_usart->BRR = this->clock.frequency_hz / a_baud_rate;
        }
        break;

        case Oversampling::unknown:
        {
            assert(Oversampling::unknown != oversampling);
        }
        break;
    }
}

void USART::set_oversampling(Oversampling a_oversampling)
{
    assert(nullptr != this->p_usart);
    assert(Oversampling::unknown != a_oversampling);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_oversampling));
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

void USART::set_word_length(Word_length a_word_length)
{
    assert(nullptr != this->p_usart);
    assert(Word_length::unknown != a_word_length);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_word_length) | USART_CR1_UE);
}

void USART::set_parity(Parity a_parity)
{
    assert(nullptr != this->p_usart);
    assert(Parity::unknown != a_parity);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_parity) | USART_CR1_UE);
}

void USART::set_stop_bits(Stop_bits a_stop_bits)
{
    assert(nullptr != this->p_usart);
    assert(Stop_bits::unknown != a_stop_bits);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR2), static_cast<uint32>(a_stop_bits));
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

void USART::set_flow_control(Flow_control a_flow_control)
{
    assert(nullptr != this->p_usart);
    assert(Flow_control::unknown != a_flow_control);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR3), static_cast<uint32>(a_flow_control));
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

USART::Oversampling USART::get_oversampling() const
{
    assert(nullptr != this->p_usart);

    return static_cast<Oversampling>(get_flag(this->p_usart->CR1, static_cast<uint32>(USART_CR1_OVER8)));
}

USART::Word_length USART::get_word_length() const
{
    assert(nullptr != this->p_usart);

    return static_cast<Word_length>(get_flag(this->p_usart->CR1, USART_CR1_M0 | USART_CR1_M1));
}

USART::Stop_bits USART::get_stop_bits() const
{
    assert(nullptr != this->p_usart);

    return static_cast<Stop_bits>(get_flag(this->p_usart->CR2, USART_CR2_STOP));
}

USART::Flow_control USART::get_flow_control() const
{
    assert(nullptr != this->p_usart);

    return static_cast<Flow_control>(get_flag(this->p_usart->CR3, USART_CR3_RTSE | USART_CR3_CTSE));
}

} // naespace stm32l452xx
} // namespace hal
} // namespace cml`

#endif // STM32L452xx