/*
    Name: USART.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <hal/stm32l452xx/usart.hpp>

//cml
#include <common/assert.hpp>
#include <hal/stm32l452xx/config.hpp>

namespace {

using namespace cml::common;
using namespace cml::hal::stm32l452xx;

void usart_1_enable(USART::Clock::Source a_clock_source)
{
    assert(a_clock_source != USART::Clock::Source::unknown);

    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART1SEL_0, RCC_CCIPR_USART1SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART1SEL, clock_source_lut[static_cast<uint32>(a_clock_source)]);
    set_flag(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);

    NVIC_SetPriority(USART1_IRQn, config::usart::_1_interrupt_priority);
    NVIC_EnableIRQ(USART1_IRQn);
}

void usart_1_disable()
{
    clear_flag(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);
    NVIC_DisableIRQ(USART1_IRQn);
}

void usart_2_enable(USART::Clock::Source a_clock_source)
{
    assert(a_clock_source != USART::Clock::Source::unknown);

    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART2SEL_0, RCC_CCIPR_USART2SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART2SEL, clock_source_lut[static_cast<uint32>(a_clock_source)]);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN);

    NVIC_SetPriority(USART2_IRQn, config::usart::_2_interrupt_priority);
    NVIC_EnableIRQ(USART2_IRQn);
}

void usart_2_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN);
    NVIC_DisableIRQ(USART2_IRQn);
}

void usart_3_enable(USART::Clock::Source a_clock_source)
{
    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART3SEL_0, RCC_CCIPR_USART3SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART3SEL, clock_source_lut[static_cast<int32>(a_clock_source)]);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN);

    NVIC_SetPriority(USART3_IRQn, config::usart::_3_interrupt_priority);
    NVIC_EnableIRQ(USART3_IRQn);
}

void usart_3_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN);
    NVIC_DisableIRQ(USART3_IRQn);
}

struct Controller
{
    USART_TypeDef* p_registers = nullptr;
    USART* p_usart_handle      = nullptr;

    void (*p_enable)(USART::Clock::Source a_clock_source) = nullptr;
    void (*p_disable)()                                   = nullptr;
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
    usart_handle_interrupt(controllers[0].p_usart_handle);
}

void USART2_IRQHandler()
{
    assert(nullptr != controllers[1].p_usart_handle);
    usart_handle_interrupt(controllers[1].p_usart_handle);
}

void USART3_IRQHandler()
{
    assert(nullptr != controllers[2].p_usart_handle);
    usart_handle_interrupt(controllers[2].p_usart_handle);
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;

void usart_handle_interrupt(USART* a_p_this)
{
    assert(nullptr != a_p_this);

    uint32 isr = a_p_this->p_usart->ISR;
    uint32 cr1 = a_p_this->p_usart->CR1;

    if (true == is_flag(isr, USART_ISR_TXE) && true == is_flag(cr1, USART_CR1_TXEIE))
    {
        byte data                    = 0;
        auto p_context               = &(a_p_this->TX_context);
        const time_tick elapsed_time = Systick::get_instance().get_counter() - p_context->start_timestamp;
        const bool procceed          = p_context->callback.p_function(&data,
                                                                      p_context->callback.p_user_data,
                                                                      elapsed_time < p_context->timeout);

        if (true == procceed)
        {
            a_p_this->p_usart->TDR = data;
        }
        else
        {
            clear_flag(&(a_p_this->p_usart->CR1), USART_CR1_TXEIE);

            p_context->callback        = { nullptr, nullptr };
            p_context->start_timestamp = 0;
            p_context->timeout         = 0;
        }
    }

    if (true == is_flag(isr, USART_ISR_RXNE) && true == is_flag(cr1, USART_CR1_RXNEIE))
    {
        auto p_context               = &(a_p_this->RX_context);
        const time_tick elapsed_time = Systick::get_instance().get_counter() - p_context->start_timestamp;
        const bool procceed          = p_context->callback.p_function(a_p_this->p_usart->RDR,
                                                                      p_context->callback.p_user_data,
                                                                      elapsed_time < p_context->timeout);

        if (false == procceed)
        {
            clear_flag(&(a_p_this->p_usart->CR1), USART_CR1_RXNEIE);

            p_context->callback        = { nullptr, nullptr };
            p_context->start_timestamp = 0;
            p_context->timeout         = 0;
        }
    }
}

bool USART::enable(const Config& a_config, const Clock &a_clock, time_tick a_timeout_ms)
{
    assert(a_config.baud_rate    != 0);
    assert(a_config.flow_control != Flow_control::unknown);
    assert(a_config.parity       != Parity::unknown);
    assert(a_config.stop_bits    != Stop_bits::unknown);
    assert(a_config.word_length  != Word_length::unknown);

    assert(Clock::Source::unknown != a_clock.source);
    assert(0                      != a_clock.frequency_hz);

    controllers[this->to_index(this->id)].p_usart_handle = this;
    this->p_usart = controllers[this->to_index(this->id)].p_registers;

    controllers[this->to_index(this->id)].p_enable(a_clock.source);

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

    return this->wait_until_isr(USART_ISR_TEACK | USART_ISR_REACK,
                                false,
                                Systick::get_instance().get_counter(),
                                a_timeout_ms);
}

void USART::disable()
{
    assert(nullptr != this->p_usart);

    this->p_usart->CR1 = 0;
    this->p_usart->CR2 = 0;
    this->p_usart->CR3 = 0;

    controllers[this->to_index(this->id)].p_disable();
    controllers[this->to_index(this->id)].p_usart_handle = nullptr;
}

void USART::write_bytes_polling(const void* a_p_data, uint32 a_data_size_in_bytes)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);

    for (decltype(a_data_size_in_bytes) i = 0; i < a_data_size_in_bytes; i++)
    {
        this->wait_until_isr(USART_ISR_TXE, false);
        this->p_usart->TDR = static_cast<const uint8*>(a_p_data)[i];
    }

    this->wait_until_isr(USART_ISR_TC, false);
    this->p_usart->ICR = USART_ICR_TCCF;
}

bool USART::write_bytes_polling(const void* a_p_data, uint32 a_data_size_in_bytes, time_tick a_timeout_ms)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);
    assert(true == Systick::get_instance().is_enabled());

    bool timeout_occured = false;
    time_tick start      = Systick::get_instance().get_counter();

    for (decltype(a_data_size_in_bytes) i = 0; i < a_data_size_in_bytes && false == timeout_occured; i++)
    {
        timeout_occured = this->wait_until_isr(USART_ISR_TXE, false, start, a_timeout_ms);

        if (false == timeout_occured)
        {
            this->p_usart->TDR = static_cast<const uint8*>(a_p_data)[i];
        }
    }

    timeout_occured = this->wait_until_isr(USART_ISR_TC, false, start, a_timeout_ms);

    return false == timeout_occured;
}

void USART::read_bytes_polling(void* a_p_data, uint32 a_data_size_in_bytes)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);

    for (decltype(a_data_size_in_bytes) i = 0; i < a_data_size_in_bytes; i++)
    {
        this->wait_until_isr(USART_ISR_RXNE, false);
        static_cast<uint8*>(a_p_data)[i] = this->p_usart->RDR;
    }
}

bool USART::read_bytes_polling(void* a_p_data, uint32 a_data_size_in_bytes, time_tick a_timeout_ms)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0);
    assert(true == Systick::get_instance().is_enabled());

    bool timeout_occured = false;
    time_tick start      = Systick::get_instance().get_counter();

    for (decltype(a_data_size_in_bytes) i = 0; i < a_data_size_in_bytes && false == timeout_occured; i++)
    {
        timeout_occured = this->wait_until_isr(USART_ISR_RXNE, false, start, a_timeout_ms);

        if (false == timeout_occured)
        {
            static_cast<uint8*>(a_p_data)[i] = this->p_usart->RDR;
        }
    }

    return false == timeout_occured;
}

void USART::write_bytes_it(const TX_callback& a_callback, time_tick a_timeout_ms)
{
    assert(true == Systick::get_instance().is_enabled());

    clear_flag(&(this->p_usart->CR1), USART_CR1_TXEIE);

    if (nullptr != a_callback.p_function)
    {
        this->TX_context.callback        = a_callback;
        this->TX_context.start_timestamp = Systick::get_instance().get_counter();
        this->TX_context.timeout         = a_timeout_ms;

        set_flag(&(this->p_usart->CR1), USART_CR1_TXEIE);
    }
    else
    {
        this->TX_context.callback        = { nullptr, nullptr };
        this->TX_context.start_timestamp = 0;
        this->TX_context.timeout         = 0;
    }
}

void USART::read_bytes_it(const RX_callback& a_callback, time_tick a_timeout_ms)
{
    assert(true == Systick::get_instance().is_enabled());

    clear_flag(&(this->p_usart->CR1), USART_CR1_RXNEIE);

    if (nullptr != a_callback.p_function)
    {
        this->RX_context.callback        = a_callback;
        this->RX_context.start_timestamp = Systick::get_instance().get_counter();;
        this->RX_context.timeout         = a_timeout_ms;

        set_flag(&(this->p_usart->CR1), USART_CR1_RXNEIE);
    }
    else
    {
        this->RX_context.callback        = { nullptr, nullptr };
        this->RX_context.start_timestamp = 0;
        this->RX_context.timeout         = 0;
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
    assert(Oversampling::unknown != a_oversampling);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_oversampling));
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

void USART::set_word_length(Word_length a_word_length)
{
    assert(Word_length::unknown != a_word_length);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_word_length) | USART_CR1_UE);
}

void USART::set_parity(Parity a_parity)
{
    assert(Parity::unknown != a_parity);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_parity) | USART_CR1_UE);
}

void USART::set_stop_bits(Stop_bits a_stop_bits)
{
    assert(Stop_bits::unknown != a_stop_bits);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR2), static_cast<uint32>(a_stop_bits));
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

void USART::set_flow_control(Flow_control a_flow_control)
{
    assert(Flow_control::unknown != a_flow_control);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR3), static_cast<uint32>(a_flow_control));
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

USART::Oversampling USART::get_oversampling() const
{
    return static_cast<Oversampling>(get_flag(this->p_usart->CR1, static_cast<uint32>(USART_CR1_OVER8)));
}

USART::Word_length USART::get_word_length() const
{
    return static_cast<Word_length>(get_flag(this->p_usart->CR1, USART_CR1_M0 | USART_CR1_M1));
}

USART::Stop_bits USART::get_stop_bits() const
{
    return static_cast<Stop_bits>(get_flag(this->p_usart->CR2, USART_CR2_STOP));
}

USART::Flow_control USART::get_flow_control() const
{
    return static_cast<Flow_control>(get_flag(this->p_usart->CR3, USART_CR3_RTSE | USART_CR3_CTSE));
}

} // naespace stm32l452xx
} // namespace hal
} // namespace cml`

#endif // STM32L452xx