/*
 *   Name: USART.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/peripherals/RS485.hpp>
#include <soc/stm32l011xx/peripherals/USART.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>

namespace {

using namespace cml;
using namespace soc;
using namespace soc::stm32l011xx::peripherals;

USART* p_usart_2 = nullptr;
RS485* p_rs485   = nullptr;

bool is_USART_ISR_error()
{
    return bit::is_any(USART2->ISR, USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
}

USART::Bus_status_flag get_bus_status_flag_from_USART_ISR()
{
    USART::Bus_status_flag ret = USART::Bus_status_flag::ok;

    if (true == bit_flag::is(USART2->ISR, USART_ISR_PE))
    {
        ret |= USART::Bus_status_flag::parity_error;
    }

    if (true == bit_flag::is(USART2->ISR, USART_ISR_FE))
    {
        ret |= USART::Bus_status_flag::framing_error;
    }

    if (true == bit_flag::is(USART2->ISR, USART_ISR_ORE))
    {
        ret |= USART::Bus_status_flag::overrun;
    }

    if (true == bit_flag::is(USART2->ISR, USART_ISR_NE))
    {
        ret |= USART::Bus_status_flag::noise_detected;
    }

    return ret;
}

void clear_USART_ISR_errors()
{
    bit_flag::set(&(USART2->ICR), USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NCF);
}

} // namespace

extern "C" {

void USART2_IRQHandler()
{
    cml_assert((nullptr != p_usart_2 && nullptr == p_rs485) || (nullptr == p_usart_2 && nullptr != p_rs485));

    if (nullptr != p_usart_2)
    {
        usart_interrupt_handler(p_usart_2);
    }
    else if (nullptr != p_rs485)
    {
        rs485_interrupt_handler(p_rs485);
    }
}

} // extern "C"

namespace soc {
namespace stm32l011xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void usart_interrupt_handler(USART* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    const uint32_t isr = USART2->ISR;
    const uint32_t cr1 = USART2->CR1;
    const uint32_t cr3 = USART2->CR3;

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_TXE) && true == bit_flag::is(cr1, USART_CR1_TXEIE))
        {
            a_p_this->transmit_callback.function(reinterpret_cast<volatile uint16_t*>(&(USART2->TDR)),
                                                 false,
                                                 a_p_this,
                                                 a_p_this->transmit_callback.p_user_data);
        }

        if (true == bit_flag::is(isr, USART_ISR_TC) && true == bit_flag::is(cr1, USART_CR1_TCIE))
        {
            a_p_this->transmit_callback.function(nullptr, true, a_p_this, a_p_this->transmit_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_RXNE) && true == bit_flag::is(cr1, USART_CR1_RXNEIE))
        {
            a_p_this->receive_callback.function(USART2->RDR, false, a_p_this, a_p_this->receive_callback.p_user_data);
        }
        else if (true == bit_flag::is(isr, USART_ISR_IDLE) && true == bit_flag::is(cr1, USART_CR1_IDLEIE))
        {
            a_p_this->receive_callback.function(0x0u, true, a_p_this, a_p_this->receive_callback.p_user_data);
            bit_flag::set(&(USART2->ICR), USART_ICR_IDLECF);
        }
    }

    if (nullptr != a_p_this->bus_status_callback.function && true == bit_flag::is(cr3, USART_CR3_EIE) &&
        true == bit_flag::is(cr1, USART_CR1_PEIE))
    {
        USART::Bus_status_flag status = get_bus_status_flag_from_USART_ISR();

        if (status != USART::Bus_status_flag::ok)
        {
            a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            clear_USART_ISR_errors();
        }
    }
}

void rs485_interrupt_handler(RS485* a_p_this)
{
    cml_assert(nullptr != a_p_this);

    const uint32_t isr = USART2->ISR;
    const uint32_t cr1 = USART2->CR1;
    const uint32_t cr3 = USART2->CR3;

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_TXE) && true == bit_flag::is(cr1, USART_CR1_TXEIE))
        {
            a_p_this->transmit_callback.function(reinterpret_cast<volatile uint16_t*>(&(USART2->TDR)),
                                                 false,
                                                 a_p_this,
                                                 a_p_this->transmit_callback.p_user_data);
        }

        if (true == bit_flag::is(isr, USART_ISR_TC) && true == bit_flag::is(cr1, USART_CR1_TCIE))
        {
            a_p_this->transmit_callback.function(nullptr, true, a_p_this, a_p_this->transmit_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == bit_flag::is(isr, USART_ISR_RXNE) && true == bit_flag::is(cr1, USART_CR1_RXNEIE))
        {
            const uint16_t rdr = USART2->RDR;

            if (false == bit_flag::is(rdr, 0x100))
            {
                a_p_this->receive_callback.function(rdr, false, a_p_this, a_p_this->receive_callback.p_user_data);
            }
        }
        else if (true == bit_flag::is(isr, USART_ISR_IDLE) && true == bit_flag::is(cr1, USART_CR1_IDLEIE))
        {
            a_p_this->receive_callback.function(0x0u, true, a_p_this, a_p_this->receive_callback.p_user_data);
            bit_flag::set(&(USART2->ICR), USART_ICR_IDLECF);
        }
    }

    if (nullptr != a_p_this->bus_status_callback.function && true == bit_flag::is(cr3, USART_CR3_EIE) &&
        true == bit_flag::is(cr1, USART_CR1_PEIE))
    {
        USART::Bus_status_flag status = get_bus_status_flag_from_USART_ISR();

        if (status != USART::Bus_status_flag::ok)
        {
            a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            clear_USART_ISR_errors();
        }
    }
}

bool USART::enable(const Config& a_config,
                   const Frame_format& a_frame_format,
                   const Clock& a_clock,
                   uint32_t a_irq_priority,
                   uint32_t s_timeout)
{
    cml_assert(0 != a_config.baud_rate);
    cml_assert(Flow_control_flag::unknown != a_config.flow_control);
    cml_assert(Stop_bits::unknown != a_config.stop_bits);
    cml_assert(Sampling_method::unknown != a_config.sampling_method);
    cml_assert(Mode_flag::unknown != a_config.mode);

    cml_assert(Parity::unknown != a_frame_format.parity);
    cml_assert(Word_length::unknown != a_frame_format.word_length);

    cml_assert(Clock::Source::unknown != a_clock.source);
    cml_assert(0 != a_clock.frequency_hz);
    cml_assert(s_timeout > 0);

    cml_assert(nullptr == p_usart_2 && nullptr == p_rs485);

    uint32_t start = system_timer::get();

    p_usart_2 = this;
    p_rs485   = nullptr;

    constexpr uint32_t clock_source_lut[] = { 0, RCC_CCIPR_USART2SEL_0, RCC_CCIPR_USART2SEL_1 };
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART2SEL, clock_source_lut[static_cast<uint32_t>(a_clock.source)]);
    bit_flag::set(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);

    NVIC_SetPriority(USART2_IRQn, a_irq_priority);
    NVIC_EnableIRQ(USART2_IRQn);

    switch (a_config.oversampling)
    {
        case Oversampling::_16: {
            USART2->BRR = a_clock.frequency_hz / a_config.baud_rate;
        }
        break;

        case Oversampling::_8: {
            const uint32_t usartdiv = 2 * a_clock.frequency_hz / a_config.baud_rate;
            USART2->BRR             = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::unknown: {
            cml_assert(a_config.oversampling != Oversampling::unknown);
        }
        break;
    }

    USART2->CR2 = static_cast<uint32_t>(a_config.stop_bits);
    USART2->CR3 = static_cast<uint32_t>(a_config.flow_control) | static_cast<uint32_t>(a_config.sampling_method);

    USART2->CR1 = static_cast<uint32_t>(a_config.oversampling) | static_cast<uint32_t>(a_config.mode) |
                  static_cast<uint32_t>(a_frame_format.parity) | static_cast<uint32_t>(a_frame_format.word_length) |
                  USART_CR1_UE;

    this->baud_rate    = a_config.baud_rate;
    this->clock        = a_clock;
    this->frame_format = a_frame_format;

    uint32_t wait_flag = (true == bit_flag::is(USART2->CR1, USART_CR1_RE) ? USART_ISR_REACK : 0) |
                         (true == bit_flag::is(USART2->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0);

    return wait_until::all_bits(&(USART2->ISR), wait_flag, false, start, s_timeout);
}

void USART::disable()
{
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    bit_flag::clear(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);
    NVIC_DisableIRQ(USART2_IRQn);

    p_usart_2 = nullptr;
}

USART::Result USART::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    bit_flag::set(&(USART2->ICR), USART_ICR_TCCF);

    uint32_t words         = 0;
    bool error             = false;
    Bus_status_flag status = Bus_status_flag::ok;

    while (false == bit_flag::is(USART2->ISR, USART_ISR_TC) && false == error)
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_TXE) && words < a_data_size_in_words)
        {
            if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
            {
                USART2->TDR = (static_cast<const uint16_t*>(a_p_data)[words++]) & 0x1FFu;
            }
            else
            {
                USART2->TDR = (static_cast<const uint8_t*>(a_p_data)[words++]) & 0xFFu;
            }
        }

        error = is_USART_ISR_error();
    }

    if (true == error)
    {
        status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { status, words };
}

USART::Result USART::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    bit_flag::set(&(USART2->ICR), USART_ICR_TCCF);

    uint32_t words         = 0;
    bool error             = false;
    Bus_status_flag status = Bus_status_flag::ok;

    while (false == bit_flag::is(USART2->ISR, USART_ISR_TC) && false == error &&
           a_timeout < various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_TXE) && words < a_data_size_in_words)
        {
            if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
            {
                USART2->TDR = (static_cast<const uint16_t*>(a_p_data)[words++]) & 0x1FFu;
            }
            else
            {
                USART2->TDR = (static_cast<const uint8_t*>(a_p_data)[words++]) & 0xFFu;
            }
        }

        error = is_USART_ISR_error();
    }

    if (true == error)
    {
        status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { status, words };
}

USART::Result USART::transmit_word(uint16_t a_word)
{
    bit_flag::set(&(USART2->ICR), USART_ICR_TCCF);

    uint32_t words         = 0;
    bool error             = false;
    Bus_status_flag status = Bus_status_flag::ok;

    while (false == bit_flag::is(USART2->ISR, USART_ISR_TC) && false == error)
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_TXE) && 0 == words)
        {
            if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
            {
                USART2->TDR = a_word & 0x1FFu;
            }
            else
            {
                USART2->TDR = a_word & 0xFFu;
            }

            words++;
        }

        error = is_USART_ISR_error();
    }

    if (true == error)
    {
        status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { status, words };
}

USART::Result USART::transmit_word(uint16_t a_word, uint32_t a_timeout)
{
    uint32_t start = system_timer::get();

    bit_flag::set(&(USART2->ICR), USART_ICR_TCCF);

    uint32_t words         = 0;
    bool error             = false;
    Bus_status_flag status = Bus_status_flag::ok;

    while (false == bit_flag::is(USART2->ISR, USART_ISR_TC) && false == error &&
           a_timeout < various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_TXE) && 0 == words)
        {
            if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
            {
                USART2->TDR = a_word & 0x1FFu;
            }
            else
            {
                USART2->TDR = a_word & 0xFFu;
            }

            words++;
        }

        error = is_USART_ISR_error();
    }

    if (true == error)
    {
        status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { status, words };
}

USART::Result USART::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    bit_flag::set(&(USART2->ICR), USART_ICR_IDLECF);

    uint32_t words         = 0;
    bool error             = false;
    Bus_status_flag status = Bus_status_flag::ok;

    while (false == bit_flag::is(USART2->ISR, USART_ISR_IDLE) && false == error)
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_RXNE))
        {
            if (words < a_data_size_in_words)
            {
                if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
                {
                    static_cast<uint16_t*>(a_p_data)[words++] = (USART2->RDR & 0x1FFu);
                }
                else
                {
                    static_cast<uint8_t*>(a_p_data)[words++] = (USART2->RDR & 0xFFu);
                }
            }
            else
            {
                bit_flag::set(&(USART2->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_USART_ISR_error();
    }

    if (true == error)
    {
        status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { status, words };
}

USART::Result USART::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    bit_flag::set(&(USART2->ICR), USART_ICR_IDLECF);

    uint32_t words         = 0;
    bool error             = false;
    Bus_status_flag status = Bus_status_flag::ok;

    while (false == bit_flag::is(USART2->ISR, USART_ISR_IDLE) && false == error &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_RXNE))
        {
            if (words < a_data_size_in_words)
            {
                if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
                {
                    static_cast<uint16_t*>(a_p_data)[words++] = (USART2->RDR & 0x1FFu);
                }
                else
                {
                    static_cast<uint8_t*>(a_p_data)[words++] = (USART2->RDR & 0xFFu);
                }
            }
            else
            {
                bit_flag::set(&(USART2->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_USART_ISR_error();
    }

    if (true == error)
    {
        status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { status, words };
}

void USART::register_transmit_callback(const Transmit_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    bit_flag::set(&(USART2->ICR), USART_ICR_TCCF);
    bit_flag::set(&(USART2->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);
}

void USART::register_receive_callback(const Receive_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    bit_flag::set(&(USART2->ICR), USART_ICR_IDLECF);
    bit_flag::set(&(USART2->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
}

void USART::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;

    bit_flag::set(&(USART2->CR1), USART_CR1_PEIE);
    bit_flag::set(&(USART2->CR3), USART_CR3_EIE);
}

void USART::unregister_transmit_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(USART2->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);

    this->transmit_callback = { nullptr, nullptr };
}

void USART::unregister_receive_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(USART2->CR1), USART_CR1_RXNEIE);

    this->receive_callback = { nullptr, nullptr };
}

void USART::unregister_bus_status_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(USART2->CR1), USART_CR1_PEIE);
    bit_flag::clear(&(USART2->CR3), USART_CR3_EIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void USART::set_baud_rate(uint32_t a_baud_rate)
{
    cml_assert(0 != a_baud_rate);

    const Oversampling oversampling = this->get_oversampling();

    switch (oversampling)
    {
        case Oversampling::_8: {
            const uint32_t usartdiv = 2 * this->clock.frequency_hz / a_baud_rate;
            USART2->BRR             = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::_16: {
            USART2->BRR = this->clock.frequency_hz / a_baud_rate;
        }
        break;

        case Oversampling::unknown: {
            cml_assert(Oversampling::unknown != oversampling);
        }
        break;
    }
}

void USART::set_oversampling(Oversampling a_oversampling)
{
    cml_assert(Oversampling::unknown != a_oversampling);

    bit_flag::clear(&(USART2->CR1), USART_CR1_UE);
    bit_flag::set(&(USART2->CR1), static_cast<uint32_t>(a_oversampling));
    bit_flag::set(&(USART2->CR1), USART_CR1_UE);
}

void USART::set_stop_bits(Stop_bits a_stop_bits)
{
    cml_assert(Stop_bits::unknown != a_stop_bits);

    bit_flag::clear(&(USART2->CR1), USART_CR1_UE);
    bit_flag::set(&(USART2->CR2), static_cast<uint32_t>(a_stop_bits));
    bit_flag::set(&(USART2->CR1), USART_CR1_UE);
}

void USART::set_flow_control(Flow_control_flag a_flow_control)
{
    cml_assert(Flow_control_flag::unknown != a_flow_control);

    bit_flag::clear(&(USART2->CR1), USART_CR1_UE);
    bit_flag::set(&(USART2->CR3), static_cast<uint32_t>(a_flow_control));
    bit_flag::set(&(USART2->CR1), USART_CR1_UE);
}

void USART::set_sampling_method(Sampling_method a_sampling_method)
{
    cml_assert(Sampling_method::unknown != a_sampling_method);

    bit_flag::clear(&(USART2->CR1), USART_CR1_UE);
    bit_flag::set(&(USART2->CR3), USART_CR3_ONEBIT, static_cast<uint32_t>(a_sampling_method));
    bit_flag::set(&(USART2->CR1), USART_CR1_UE);
}

void USART::set_frame_format(const Frame_format& a_frame_format)
{
    cml_assert(USART::Word_length::unknown != a_frame_format.word_length);
    cml_assert(USART::Parity::unknown != a_frame_format.parity);

    bit_flag::clear(&(USART2->CR1), USART_CR1_UE);
    bit_flag::set(&(USART2->CR1),
                  USART_CR1_PCE | USART_CR1_M,
                  static_cast<uint32_t>(a_frame_format.parity) | static_cast<uint32_t>(a_frame_format.word_length) |
                      USART_CR1_UE);

    this->frame_format = a_frame_format;
}

bool USART::set_mode(Mode_flag a_mode, uint32_t a_timeout)
{
    cml_assert(Mode_flag::unknown != a_mode);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    bit_flag::set(&(USART2->CR1), USART_CR1_TE | USART_CR1_RE, static_cast<uint32_t>(a_mode));

    uint32_t wait_flag = (true == bit_flag::is(USART2->CR1, USART_CR1_RE) ? USART_ISR_REACK : 0) |
                         (true == bit_flag::is(USART2->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0);

    return wait_until::all_bits(&(USART2->ISR), wait_flag, false, start, a_timeout);
}

USART::Oversampling USART::get_oversampling() const
{
    return static_cast<Oversampling>(bit_flag::get(USART2->CR1, static_cast<uint32_t>(USART_CR1_OVER8)));
}

USART::Stop_bits USART::get_stop_bits() const
{
    return static_cast<Stop_bits>(bit_flag::get(USART2->CR2, USART_CR2_STOP));
}

USART::Flow_control_flag USART::get_flow_control() const
{
    return static_cast<Flow_control_flag>(bit_flag::get(USART2->CR3, USART_CR3_RTSE | USART_CR3_CTSE));
}

USART::Sampling_method USART::get_sampling_method() const
{
    return static_cast<Sampling_method>(bit_flag::get(USART2->CR3, USART_CR3_ONEBIT));
}

USART::Mode_flag USART::get_mode() const
{
    return static_cast<Mode_flag>(bit_flag::get(USART2->CR1, USART_CR1_TE | USART_CR1_RE));
}

bool USART::is_enabled() const
{
    return true == bit_flag::is(USART2->CR1, USART_CR1_UE);
}

bool RS485::enable(const Config& a_config,
                   const USART::Clock& a_clock,
                   GPIO::Out::Pin* a_p_flow_control_pin,
                   uint32_t a_irq_priority,
                   uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_flow_control_pin);
    cml_assert(0 != a_config.baud_rate);
    cml_assert(Stop_bits::unknown != a_config.stop_bits);

    cml_assert(USART::Clock::Source::unknown != a_clock.source);
    cml_assert(0 != a_clock.frequency_hz);
    cml_assert(a_timeout > 0);

    cml_assert(nullptr == p_usart_2 && nullptr == p_rs485);

    uint32_t start = system_timer::get();

    p_rs485   = this;
    p_usart_2 = nullptr;

    constexpr uint32_t clock_source_lut[] = { 0, RCC_CCIPR_USART2SEL_0, RCC_CCIPR_USART2SEL_1 };
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART2SEL, clock_source_lut[static_cast<uint32_t>(a_clock.source)]);
    bit_flag::set(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);

    NVIC_SetPriority(USART2_IRQn, a_irq_priority);
    NVIC_EnableIRQ(USART2_IRQn);

    switch (a_config.oversampling)
    {
        case Oversampling::_16: {
            USART2->BRR = a_clock.frequency_hz / a_config.baud_rate;
        }
        break;

        case Oversampling::_8: {
            const uint32_t usartdiv = 2 * a_clock.frequency_hz / a_config.baud_rate;
            USART2->BRR             = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::unknown: {
            cml_assert(a_config.oversampling != Oversampling::unknown);
        }
        break;
    }

    USART2->CR3 = USART_CR3_ONEBIT;
    USART2->CR2 = static_cast<uint32_t>(a_config.stop_bits) | (a_config.address << USART_CR2_ADD_Pos) | USART_CR2_ADDM7;

    USART2->CR1 = static_cast<uint32_t>(a_config.oversampling) | USART_CR1_M0 | USART_CR1_UE | USART_CR1_TE |
                  USART_CR1_RE | USART_CR1_MME | USART_CR1_WAKE;

    USART2->RQR = USART_RQR_MMRQ;

    this->p_flow_control_pin = a_p_flow_control_pin;
    this->baud_rate          = a_config.baud_rate;
    this->clock              = a_clock;

    bool ret = wait_until::all_bits(
        &(USART2->ISR), USART_ISR_TEACK | USART_ISR_REACK | USART_ISR_RWU, false, start, a_timeout);

    if (true == ret)
    {
        this->p_flow_control_pin->set_level(GPIO::Level::low);
    }
    else
    {
        this->disable();
    }

    return ret;
}

void RS485::disable()
{
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->RQR = 0;

    bit_flag::clear(&(RCC->APB1ENR), RCC_APB1ENR_USART2EN);
    NVIC_DisableIRQ(USART2_IRQn);

    p_rs485 = nullptr;
}

RS485::Result RS485::transmit_bytes_polling(uint8_t a_address, const void* a_p_data, uint32_t a_data_size_in_words)
{
    cml_assert(a_address <= 0x7F);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    bit_flag::set(&(USART2->ICR), USART_ICR_TCCF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    this->p_flow_control_pin->set_level(GPIO::Level::high);

    while (false == bit_flag::is(USART2->ISR, USART_ISR_TC) && false == error)
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_TXE))
        {
            if (words == 0)
            {
                USART2->TDR = (static_cast<uint16_t>(a_address) | static_cast<uint16_t>(0x100u));
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                USART2->TDR = (static_cast<const uint8_t*>(a_p_data)[words - 1]);
                words++;
            }
        }

        error = is_USART_ISR_error();
    }

    this->p_flow_control_pin->set_level(GPIO::Level::low);

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { bus_status, words };
}

RS485::Result RS485::transmit_bytes_polling(uint8_t a_address,
                                            const void* a_p_data,
                                            uint32_t a_data_size_in_words,
                                            uint32_t a_timeout)
{
    cml_assert(a_address <= 0x7F);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    bit_flag::set(&(USART2->ICR), USART_ICR_TCCF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    this->p_flow_control_pin->set_level(GPIO::Level::high);

    while (false == bit_flag::is(USART2->ISR, USART_ISR_TC) && false == error &&
           a_timeout < various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_TXE))
        {
            if (words == 0)
            {
                USART2->TDR = (static_cast<uint16_t>(a_address) | static_cast<uint16_t>(0x100u));
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                USART2->TDR = (static_cast<const uint8_t*>(a_p_data)[words - 1]);
                words++;
            }
        }

        error = is_USART_ISR_error();
    }

    this->p_flow_control_pin->set_level(GPIO::Level::low);

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { bus_status, words };
}

RS485::Result RS485::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    bit_flag::set(&(USART2->ICR), USART_ICR_IDLECF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == bit_flag::is(USART2->ISR, USART_ISR_IDLE) && false == error)
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_RXNE))
        {
            if (0 == words)
            {
                bit_flag::set(&(USART2->RQR), USART_RQR_RXFRQ);
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                static_cast<uint8_t*>(a_p_data)[words - 1] = (USART2->RDR & 0xFFu);
                words++;
            }
            else
            {
                bit_flag::set(&(USART2->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_USART_ISR_error();
    }

    bit_flag::set(&(USART2->ICR), USART_ICR_CMCF);

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { bus_status, words };
}

RS485::Result RS485::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

    bit_flag::set(&(USART2->ICR), USART_ICR_IDLECF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == bit_flag::is(USART2->ISR, USART_ISR_IDLE) && false == error &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(USART2->ISR, USART_ISR_RXNE))
        {
            if (0 == words)
            {
                bit_flag::set(&(USART2->RQR), USART_RQR_RXFRQ);
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                static_cast<uint8_t*>(a_p_data)[words - 1] = (USART2->RDR & 0xFFu);
                words++;
            }
            else
            {
                bit_flag::set(&(USART2->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_USART_ISR_error();
    }

    bit_flag::set(&(USART2->ICR), USART_ICR_CMCF);

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR();
        clear_USART_ISR_errors();
    }

    return { bus_status, words };
}

void RS485::register_transmit_callback(const Transmit_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    bit_flag::set(&(USART2->ICR), USART_ICR_TCCF);
    bit_flag::set(&(USART2->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);

    this->p_flow_control_pin->set_level(GPIO::Level::high);
}

void RS485::register_receive_callback(const Receive_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    bit_flag::set(&(USART2->ICR), USART_ICR_IDLECF);
    bit_flag::set(&(USART2->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
}

void RS485::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;

    bit_flag::set(&(USART2->CR1), USART_CR1_PEIE);
    bit_flag::set(&(USART2->CR3), USART_CR3_EIE);
}

void RS485::unregister_transmit_callback()
{
    Interrupt_guard guard;

    this->p_flow_control_pin->set_level(GPIO::Level::low);

    bit_flag::clear(&(USART2->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);

    this->transmit_callback = { nullptr, nullptr };
}

void RS485::unregister_receive_callback()
{
    Interrupt_guard guard;

    bit_flag::set(&(USART2->ICR), USART_ICR_CMCF);
    bit_flag::clear(&(USART2->CR1), USART_CR1_RXNEIE);

    this->receive_callback = { nullptr, nullptr };
}

void RS485::unregister_bus_status_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(USART2->CR1), USART_CR1_PEIE);
    bit_flag::clear(&(USART2->CR3), USART_CR3_EIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void RS485::set_baud_rate(uint32_t a_baud_rate)
{
    cml_assert(0 != a_baud_rate);

    const Oversampling oversampling = this->get_oversampling();

    switch (oversampling)
    {
        case Oversampling::_8: {
            const uint32_t usartdiv = 2 * this->clock.frequency_hz / a_baud_rate;
            USART2->BRR             = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::_16: {
            USART2->BRR = this->clock.frequency_hz / a_baud_rate;
        }
        break;

        case Oversampling::unknown: {
            cml_assert(Oversampling::unknown != oversampling);
        }
        break;
    }
}

void RS485::set_oversampling(Oversampling a_oversampling)
{
    cml_assert(Oversampling::unknown != a_oversampling);

    bit_flag::clear(&(USART2->CR1), USART_CR1_UE);
    bit_flag::set(&(USART2->CR1), static_cast<uint32_t>(a_oversampling));
    bit_flag::set(&(USART2->CR1), USART_CR1_UE);
}

void RS485::set_stop_bits(Stop_bits a_stop_bits)
{
    cml_assert(Stop_bits::unknown != a_stop_bits);

    bit_flag::clear(&(USART2->CR1), USART_CR1_UE);
    bit_flag::set(&(USART2->CR2), static_cast<uint32_t>(a_stop_bits));
    bit_flag::set(&(USART2->CR1), USART_CR1_UE);
}

bool RS485::is_enabled() const
{
    return bit_flag::is(USART2->CR1, USART_CR1_UE);
}

RS485::Oversampling RS485::get_oversampling() const
{
    return static_cast<Oversampling>(bit_flag::get(USART2->CR1, static_cast<uint32_t>(USART_CR1_OVER8)));
}

RS485::Stop_bits RS485::get_stop_bits() const
{
    return static_cast<Stop_bits>(bit_flag::get(USART2->CR2, USART_CR2_STOP));
}

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx