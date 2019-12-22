/*
    Name: usart.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <hal/stm32l452xx/usart.hpp>

//cml
#include <common/assert.hpp>
#include <common/bit.hpp>
#include <hal/stm32l452xx/config.hpp>

namespace {

using namespace cml::common;
using namespace cml::hal::stm32l452xx;

void usart_1_enable(c_usart::s_clock::e_source a_clock_source)
{
    _assert(a_clock_source != c_usart::s_clock::e_source::unknown);

    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART1SEL_0, RCC_CCIPR_USART1SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART1SEL, clock_source_lut[static_cast<uint32>(a_clock_source)]);
    set_flag(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);

    NVIC_SetPriority(USART1_IRQn, s_config::s_usart::_1_interrupt_priority);
    NVIC_EnableIRQ(USART1_IRQn);
}

void usart_1_disable()
{
    clear_flag(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);
    NVIC_DisableIRQ(USART1_IRQn);
}

void usart_2_enable(c_usart::s_clock::e_source a_clock_source)
{
    _assert(a_clock_source != c_usart::s_clock::e_source::unknown);

    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART2SEL_0, RCC_CCIPR_USART2SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART2SEL, clock_source_lut[static_cast<uint32>(a_clock_source)]);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN);

    NVIC_SetPriority(USART2_IRQn, s_config::s_usart::_2_interrupt_priority);
    NVIC_EnableIRQ(USART2_IRQn);
}

void usart_2_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN);
    NVIC_DisableIRQ(USART2_IRQn);
}

void usart_3_enable(c_usart::s_clock::e_source a_clock_source)
{
    constexpr uint32 clock_source_lut[] = { 0, RCC_CCIPR_USART3SEL_0, RCC_CCIPR_USART3SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART3SEL, clock_source_lut[static_cast<int32>(a_clock_source)]);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN);

    NVIC_SetPriority(USART3_IRQn, s_config::s_usart::_3_interrupt_priority);
    NVIC_EnableIRQ(USART3_IRQn);
}

void usart_3_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN);
    NVIC_DisableIRQ(USART3_IRQn);
}

struct s_ll_config_data
{
    USART_TypeDef* p_registers = nullptr;
    c_usart* p_usart_handle    = nullptr;

    void (*p_enable)(c_usart::s_clock::e_source a_clock_source)  = nullptr;
    void (*p_disable)()                                          = nullptr;
};

s_ll_config_data ll_configs[] =
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
    _assert(nullptr != ll_configs[0].p_usart_handle);
    usart_handle_interrupt(ll_configs[0].p_usart_handle);
}

void USART2_IRQHandler()
{
    _assert(nullptr != ll_configs[1].p_usart_handle);
    usart_handle_interrupt(ll_configs[1].p_usart_handle);
}

void USART3_IRQHandler()
{
    _assert(nullptr != ll_configs[2].p_usart_handle);
    usart_handle_interrupt(ll_configs[2].p_usart_handle);
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;

void usart_handle_interrupt(c_usart* a_p_this)
{
    _assert(nullptr != a_p_this);

    uint32 isr = a_p_this->p_usart->ISR;
    uint32 cr1 = a_p_this->p_usart->CR1;

    if (true == is_flag(isr, USART_ISR_TXE) && true == is_flag(cr1, USART_CR1_TXEIE))
    {
        byte data                    = 0;
        auto p_context               = &(a_p_this->tx_context);
        const time_tick elapsed_time = c_systick::get_instance().get_counter() - p_context->start_timestamp;
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
        auto p_context               = &(a_p_this->rx_context);
        const time_tick elapsed_time = c_systick::get_instance().get_counter() - p_context->start_timestamp;
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

void c_usart::enable(const s_config& a_config, const s_clock &a_clock)
{
    _assert(a_config.baud_rate    != e_baud_rate::unknown);
    _assert(a_config.flow_control != e_flow_control::unknown);
    _assert(a_config.mode         != e_mode::unknown);
    _assert(a_config.oversampling != e_oversampling::unknown);
    _assert(a_config.parity       != e_parity::unknown);
    _assert(a_config.stop_bits    != e_stop_bits::unknown);
    _assert(a_config.word_length  != e_word_length::unknown);

    _assert(a_clock.source != s_clock::e_source::unknown);
    _assert(0 != a_clock.frequency_hz);

    ll_configs[this->to_index(this->periph)].p_usart_handle = this;
    this->p_usart = ll_configs[this->to_index(this->periph)].p_registers;

    ll_configs[this->to_index(this->periph)].p_enable(a_clock.source);

    this->p_usart->CR1 = 0;
    this->p_usart->CR2 = 0;
    this->p_usart->CR3 = 0;

    this->p_usart->BRR = a_clock.frequency_hz / static_cast<uint32>(a_config.baud_rate);
    this->p_usart->CR2 = static_cast<uint32>(a_config.stop_bits);
    this->p_usart->CR3 = static_cast<uint32>(a_config.flow_control);

    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_config.word_length)  |
                                    static_cast<uint32>(a_config.oversampling) |
                                    static_cast<uint32>(a_config.parity)       |
                                    static_cast<uint32>(a_config.mode)         |
                                    USART_CR1_UE);

    this->baud_rate = a_config.baud_rate;
}

void c_usart::disable()
{
    _assert(nullptr != this->p_usart);

    this->p_usart->CR1 = 0;
    this->p_usart->CR2 = 0;
    this->p_usart->CR3 = 0;

    ll_configs[this->to_index(this->periph)].p_disable();
    ll_configs[this->to_index(this->periph)].p_usart_handle = nullptr;
}

void c_usart::write_bytes_polling(const void* a_p_data, uint32 a_data_length)
{
    _assert(nullptr != a_p_data);
    _assert(a_data_length > 0);

    for (decltype(a_data_length) i = 0; i < a_data_length; i++)
    {
        while (false == is_flag(this->p_usart->ISR, USART_ISR_TXE));
        this->p_usart->TDR = static_cast<const uint8*>(a_p_data)[i];
    }
}

bool c_usart::write_bytes_polling(const void* a_p_data, uint32 a_data_length, time_tick a_timeout)
{
    _assert(nullptr != a_p_data);
    _assert(a_data_length > 0);
    _assert(true == c_systick::get_instance().is_enabled());

    bool timeout_occured = false;
    time_tick start      = c_systick::get_instance().get_counter();

    for (decltype(a_data_length) i = 0; i < a_data_length && false == timeout_occured; i++)
    {
        while (false == is_flag(this->p_usart->ISR, USART_ISR_TXE) && false == timeout_occured)
        {
            timeout_occured = c_systick::get_instance().get_counter() - start >= a_timeout;
        }

        if (false == timeout_occured)
        {
            this->p_usart->TDR = static_cast<const uint8*>(a_p_data)[i];
        }
    }

    return false == timeout_occured;
}

void c_usart::read_bytes_polling(void* a_p_data, uint32 a_data_length)
{
    _assert(nullptr != a_p_data);
    _assert(a_data_length > 0);

    for (decltype(a_data_length) i = 0; i < a_data_length; i++)
    {
        while (false == is_flag(this->p_usart->ISR, USART_ISR_RXNE));
        static_cast<uint8*>(a_p_data)[i] = this->p_usart->RDR;
    }
}

bool c_usart::read_bytes_polling(void* a_p_data, uint32 a_data_length, time_tick a_timeout)
{
    _assert(nullptr != a_p_data);
    _assert(a_data_length > 0);
    _assert(true == c_systick::get_instance().is_enabled());

    bool timeout_occured = false;
    time_tick start      = c_systick::get_instance().get_counter();

    for (decltype(a_data_length) i = 0; i < a_data_length && false == timeout_occured; i++)
    {
        while (false == is_flag(this->p_usart->ISR, USART_ISR_RXNE) && false == timeout_occured)
        {
            timeout_occured = c_systick::get_instance().get_counter() - start >= a_timeout;
        }

        if (false == timeout_occured)
        {
            static_cast<uint8*>(a_p_data)[i] = this->p_usart->RDR;
        }
    }

    return false == timeout_occured;
}

void c_usart::write_bytes_it(const s_tx_callback& a_callback, time_tick a_timeout)
{
    _assert(true == c_systick::get_instance().is_enabled());

    clear_flag(&(this->p_usart->CR1), USART_CR1_TXEIE);

    if (nullptr != a_callback.p_function)
    {
        this->tx_context.callback        = a_callback;
        this->tx_context.start_timestamp = c_systick::get_instance().get_counter();
        this->tx_context.timeout         = a_timeout;

        set_flag(&(this->p_usart->CR1), USART_CR1_TXEIE);
    }
    else
    {
        this->tx_context.callback        = { nullptr, nullptr };
        this->tx_context.start_timestamp = 0;
        this->tx_context.timeout         = 0;
    }
}

void c_usart::read_bytes_it(const s_rx_callback& a_callback, time_tick a_timeout)
{
    _assert(true == c_systick::get_instance().is_enabled());

    clear_flag(&(this->p_usart->CR1), USART_CR1_RXNEIE);

    if (nullptr != a_callback.p_function)
    {
        this->rx_context.callback        = a_callback;
        this->rx_context.start_timestamp = c_systick::get_instance().get_counter();;
        this->rx_context.timeout         = a_timeout;

        set_flag(&(this->p_usart->CR1), USART_CR1_RXNEIE);
    }
    else
    {
        this->rx_context.callback        = { nullptr, nullptr };
        this->rx_context.start_timestamp = 0;
        this->rx_context.timeout         = 0;
    }
}

void c_usart::set_baud_rate(e_baud_rate a_baud_rate)
{
    _assert(e_baud_rate::unknown != a_baud_rate);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    this->p_usart->BRR = SystemCoreClock / static_cast<uint32>(a_baud_rate);
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

void c_usart::set_oversampling(e_oversampling a_oversampling)
{
    _assert(e_oversampling::unknown != a_oversampling);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_oversampling));
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

void c_usart::set_word_length(e_word_length a_word_length)
{
    _assert(e_word_length::unknown != a_word_length);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_word_length) | USART_CR1_UE);
}

void c_usart::set_parity(e_parity a_parity)
{
    _assert(e_parity::unknown != a_parity);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_parity) | USART_CR1_UE);
}

void c_usart::set_stop_bits(e_stop_bits a_stop_bits)
{
    _assert(e_stop_bits::unknown != a_stop_bits);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR2), static_cast<uint32>(a_stop_bits));
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

void c_usart::set_flow_control(e_flow_control a_flow_control)
{
    _assert(e_flow_control::unknown != a_flow_control);

    clear_flag(&(this->p_usart->CR1), USART_CR1_UE);
    set_flag(&(this->p_usart->CR3), static_cast<uint32>(a_flow_control));
    set_flag(&(this->p_usart->CR1), USART_CR1_UE);
}

void c_usart::set_mode(e_mode a_mode)
{
    _assert(e_mode::unknown != a_mode);

    clear_flag(&(this->p_usart->CR1), USART_CR1_RE | USART_CR1_TE);
    set_flag(&(this->p_usart->CR1), static_cast<uint32>(a_mode));
}

c_usart::e_baud_rate c_usart::get_baud_rate() const
{
    return this->baud_rate;
}

c_usart::e_oversampling c_usart::get_oversampling() const
{
    return static_cast<e_oversampling>(get_flag(this->p_usart->CR1, static_cast<uint32>(USART_CR1_OVER8)));
}

c_usart::e_word_length c_usart::get_word_length() const
{
    return static_cast<e_word_length>(get_flag(this->p_usart->CR1, USART_CR1_M0 | USART_CR1_M1));
}

c_usart::e_stop_bits c_usart::get_stop_bits() const
{
    return static_cast<e_stop_bits>(get_flag(this->p_usart->CR2, USART_CR2_STOP));
}

c_usart::e_flow_control c_usart::get_flow_control() const
{
    return static_cast<e_flow_control>(get_flag(this->p_usart->CR3, USART_CR3_RTSE | USART_CR3_CTSE));
}

c_usart::e_mode c_usart::get_mode() const
{
    return static_cast<e_mode>(get_flag(this->p_usart->CR1, (USART_CR1_TE_Msk | USART_CR1_RE_Msk)));
}

} // naespace stm32l452xx
} // namespace hal
} // namespace cml`

#endif // STM32L452xx