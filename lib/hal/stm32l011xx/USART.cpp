/*
    Name: USART.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

//this
#include <hal/stm32l011xx/USART.hpp>

//cml
#include <hal/stm32l011xx/config.hpp>
#include <debug/assert.hpp>
#include <utils/sleep.hpp>

namespace {

using namespace cml::common;
using namespace cml::hal;
using namespace cml::hal::stm32l011xx;

USART* p_usart_2 = nullptr;

bool is_timeout(time_tick a_start, time_tick a_timeout)
{
    return time_tick_infinity == a_timeout ? false : time_tick_diff(systick::get_counter(), a_start) < a_timeout;
}

} // namespace ::

extern "C"
{

void USART2_IRQHandler()
{
    assert(nullptr != p_usart_2);
    usart_handle_interrupt(p_usart_2);
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l011xx {

using namespace cml::common;
using namespace cml::utils;

void usart_handle_interrupt(USART* a_p_this)
{
    assert(nullptr != a_p_this);

    uint32 isr = USART2->ISR;
    uint32 cr1 = USART2->CR1;

    if (true == is_flag(isr, USART_ISR_TXE) && true == is_flag(cr1, USART_CR1_TXEIE))
    {
        byte data           = 0;
        auto p_context      = &(a_p_this->tx_context);
        const bool procceed = p_context->callback.p_function(&data,
                                                             p_context->callback.p_user_data,
                                                             is_timeout(p_context->start_timestamp,
                                                                        p_context->timeout));

        if (true == procceed)
        {
            USART2->TDR = data;
        }
        else
        {
            clear_flag(&(USART2->CR1), USART_CR1_TXEIE);

            p_context->callback        = { nullptr, nullptr };
            p_context->start_timestamp = 0;
            p_context->timeout         = 0;
        }
    }

    if (true == is_flag(isr, USART_ISR_RXNE) && true == is_flag(cr1, USART_CR1_RXNEIE))
    {
        auto p_context               = &(a_p_this->rx_context);
        const bool procceed          = p_context->callback.p_function(USART2->RDR,
                                                                      p_context->callback.p_user_data,
                                                                      is_timeout(p_context->start_timestamp,
                                                                                 p_context->timeout));

        if (false == procceed)
        {
            clear_flag(&(USART2->CR1), USART_CR1_RXNEIE);

            p_context->callback        = { nullptr, nullptr };
            p_context->start_timestamp = 0;
            p_context->timeout         = 0;
        }
    }
}

bool USART::enable(const Config& a_config, const Clock &a_clock, uint32 a_irq_priority, time_tick a_timeout_ms)
{
    assert(0                     != a_config.baud_rate);
    assert(Flow_control::unknown != a_config.flow_control);
    assert(Parity::unknown       != a_config.parity);
    assert(Stop_bits::unknown    != a_config.stop_bits);
    assert(Word_length::unknown  != a_config.word_length);

    assert(Clock::Source::unknown != a_clock.source);
    assert(0                      != a_clock.frequency_hz);

    time_tick start = systick::get_counter();

    this->disable();

    p_usart_2 = this;

    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART2SEL_0, RCC_CCIPR_USART2SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART2SEL, clock_source_lut[static_cast<uint32>(a_clock.source)]);
    set_flag(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);

    NVIC_SetPriority(USART2_IRQn, a_irq_priority);
    NVIC_EnableIRQ(USART2_IRQn);

    USART2->CR2 = static_cast<uint32>(a_config.stop_bits);
    USART2->CR3 = static_cast<uint32>(a_config.flow_control);

    switch (a_config.oversampling)
    {
        case Oversampling::_16:
        {
            USART2->BRR = a_clock.frequency_hz / a_config.baud_rate;
        }
        break;

        case Oversampling::_8:
        {
            uint32 usartdiv = 2 * a_clock.frequency_hz / a_config.baud_rate;
            USART2->BRR = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::unknown:
        {
            assert(a_config.oversampling != Oversampling::unknown);
        }
        break;
    }

    set_flag(&(USART2->CR1), static_cast<uint32>(a_config.word_length)  |
                             static_cast<uint32>(a_config.oversampling) |
                             static_cast<uint32>(a_config.parity));

    set_flag(&(USART2->CR1), USART_CR1_UE);
    set_flag(&(USART2->CR1), USART_CR1_RE | USART_CR1_TE);

    this->baud_rate = a_config.baud_rate;
    this->clock     = a_clock;

    bool ret = sleep::until(&(USART2->ISR), USART_ISR_REACK | USART_ISR_TEACK, false, start, a_timeout_ms);

    if (false == ret)
    {
        this->disable();
    }

    return ret;
}

void USART::disable()
{
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    clear_flag(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);
    NVIC_DisableIRQ(USART2_IRQn);

    p_usart_2 = nullptr;
}

void USART::write_bytes_polling(const void* a_p_data, uint32 a_data_size_in_bytes)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);

    for (decltype(a_data_size_in_bytes) i = 0; i < a_data_size_in_bytes; i++)
    {
        sleep::until(&(USART2->ISR), USART_ISR_TXE, false);
        USART2->TDR = static_cast<const uint8*>(a_p_data)[i];
    }

    sleep::until(&(USART2->ISR), USART_ISR_TC, false);
    USART2->ICR = USART_ICR_TCCF;
}

bool USART::write_bytes_polling(const void* a_p_data, uint32 a_data_size_in_bytes, time_tick a_timeout_ms)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);
    assert(true == systick::is_enabled());

    bool ret        = true;
    time_tick start = systick::get_counter();

    set_flag(&(USART2->CR1), USART_CR1_TE);

    for (decltype(a_data_size_in_bytes) i = 0; i < a_data_size_in_bytes && true == ret; i++)
    {
        ret = sleep::until(&(USART2->ISR), USART_ISR_TXE, false, start, a_timeout_ms);

        if (false == ret)
        {
            USART2->TDR = static_cast<const uint8*>(a_p_data)[i];
        }
    }

    if (true == ret)
    {
        ret = sleep::until(&(USART2->ISR), USART_ISR_TC, false, start, a_timeout_ms);
    }

    return ret;
}

void USART::read_bytes_polling(void* a_p_data, uint32 a_data_size_in_bytes)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);

    for (decltype(a_data_size_in_bytes) i = 0; i < a_data_size_in_bytes; i++)
    {
        sleep::until(&(USART2->ISR), USART_ISR_RXNE, false);
        static_cast<uint8*>(a_p_data)[i] = USART2->RDR;
    }
}

bool USART::read_bytes_polling(void* a_p_data, uint32 a_data_size_in_bytes, time_tick a_timeout_ms)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);
    assert(true == systick::is_enabled());

    bool ret        = false;
    time_tick start = systick::get_counter();

    for (decltype(a_data_size_in_bytes) i = 0; i < a_data_size_in_bytes && true == ret; i++)
    {
        ret = sleep::until(&(USART2->ISR), USART_ISR_RXNE, false, start, a_timeout_ms);

        if (false == ret)
        {
            static_cast<uint8*>(a_p_data)[i] = USART2->RDR;
        }
    }

    return ret;
}

void USART::write_bytes_it(const TX_callback& a_callback, time_tick a_timeout_ms)
{
    assert(true == systick::is_enabled());

    clear_flag(&(USART2->CR1), USART_CR1_TXEIE);

    if (nullptr != a_callback.p_function)
    {
        this->tx_context.callback        = a_callback;
        this->tx_context.start_timestamp = systick::get_counter();
        this->tx_context.timeout         = a_timeout_ms;

        set_flag(&(USART2->CR1), USART_CR1_TXEIE);
    }
    else
    {
        this->tx_context.callback        = { nullptr, nullptr };
        this->tx_context.start_timestamp = 0;
        this->tx_context.timeout         = 0;
    }
}

void USART::read_bytes_it(const RX_callback& a_callback, time_tick a_timeout_ms)
{
    assert(true == systick::is_enabled());

    clear_flag(&(USART2->CR1), USART_CR1_RXNEIE);

    if (nullptr != a_callback.p_function)
    {
        this->rx_context.callback        = a_callback;
        this->rx_context.start_timestamp = systick::get_counter();;
        this->rx_context.timeout         = a_timeout_ms;

        set_flag(&(USART2->CR1), USART_CR1_RXNEIE);
    }
    else
    {
        this->rx_context.callback        = { nullptr, nullptr };
        this->rx_context.start_timestamp = 0;
        this->rx_context.timeout         = 0;
    }
}

void USART::set_baud_rate(uint32 a_baud_rate)
{
    assert(0 != a_baud_rate);

    const Oversampling oversampling = this->get_oversampling();

    switch (oversampling)
    {
        case Oversampling::_8:
        {
            uint32 usartdiv = 2 * this->clock.frequency_hz / a_baud_rate;
            USART2->BRR = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::_16:
        {
            USART2->BRR = this->clock.frequency_hz / a_baud_rate;
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
    assert(Oversampling::unknown != a_oversampling);

    clear_flag(&(USART2->CR1), USART_CR1_UE);
    set_flag(&(USART2->CR1), static_cast<uint32>(a_oversampling));
    set_flag(&(USART2->CR1), USART_CR1_UE);
}

void USART::set_word_length(Word_length a_word_length)
{
    assert(Word_length::unknown != a_word_length);

    clear_flag(&(USART2->CR1), USART_CR1_UE);
    set_flag(&(USART2->CR1), static_cast<uint32>(a_word_length) | USART_CR1_UE);
}

void USART::set_parity(Parity a_parity)
{
    assert(Parity::unknown != a_parity);

    clear_flag(&(USART2->CR1), USART_CR1_UE);
    set_flag(&(USART2->CR1), static_cast<uint32>(a_parity) | USART_CR1_UE);
}

void USART::set_stop_bits(Stop_bits a_stop_bits)
{
    assert(Stop_bits::unknown != a_stop_bits);

    clear_flag(&(USART2->CR1), USART_CR1_UE);
    set_flag(&(USART2->CR2), static_cast<uint32>(a_stop_bits));
    set_flag(&(USART2->CR1), USART_CR1_UE);
}

void USART::set_flow_control(Flow_control a_flow_control)
{
    assert(Flow_control::unknown != a_flow_control);

    clear_flag(&(USART2->CR1), USART_CR1_UE);
    set_flag(&(USART2->CR3), static_cast<uint32>(a_flow_control));
    set_flag(&(USART2->CR1), USART_CR1_UE);
}

USART::Oversampling USART::get_oversampling() const
{
    return static_cast<Oversampling>(get_flag(USART2->CR1, static_cast<uint32>(USART_CR1_OVER8)));
}

USART::Word_length USART::get_word_length() const
{
    return static_cast<Word_length>(get_flag(USART2->CR1, USART_CR1_M0 | USART_CR1_M1));
}

USART::Stop_bits USART::get_stop_bits() const
{
    return static_cast<Stop_bits>(get_flag(USART2->CR2, USART_CR2_STOP));
}

USART::Flow_control USART::get_flow_control() const
{
    return static_cast<Flow_control>(get_flag(USART2->CR3, USART_CR3_RTSE | USART_CR3_CTSE));
}

bool USART::is_rx_it_enabled() const
{
    return is_flag(USART2->CR1, USART_CR1_RXNEIE);
}

bool USART::is_tx_it_enabled() const
{
    return is_flag(USART2->CR1, USART_CR1_TXEIE);
}

} // naespace stm32l011xx
} // namespace hal
} // namespace cml`

#endif // STM32L011xx