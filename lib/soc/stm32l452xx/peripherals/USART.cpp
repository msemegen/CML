/*
    Name: USART.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/peripherals/GPIO.hpp>
#include <soc/stm32l452xx/peripherals/RS485.hpp>
#include <soc/stm32l452xx/peripherals/USART.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/counter.hpp>

// cml
#include <cml/debug/assert.hpp>
#include <cml/utils/wait.hpp>

namespace {

using namespace cml;
using namespace soc;
using namespace soc::stm32l452xx::peripherals;

struct Controller
{
    using Enable_function  = void (*)(USART::Clock::Source a_clock_source, uint32_t a_irq_priority);
    using Disable_function = void (*)();

    USART_TypeDef* p_registers = nullptr;
    USART* p_usart_handle      = nullptr;
    RS485* p_rs485_handle      = nullptr;

    Enable_function enable   = nullptr;
    Disable_function disable = nullptr;
};

void usart_1_enable(USART::Clock::Source a_clock_source, uint32_t a_irq_priority)
{
    assert(a_clock_source != USART::Clock::Source::unknown);

    constexpr uint32_t clock_source_lut[] = { 0, RCC_CCIPR_USART1SEL_0, RCC_CCIPR_USART1SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART1SEL, clock_source_lut[static_cast<uint32_t>(a_clock_source)]);
    set_flag(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);

    NVIC_SetPriority(USART1_IRQn, a_irq_priority);
    NVIC_EnableIRQ(USART1_IRQn);
}

void usart_1_disable()
{
    clear_flag(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);
    NVIC_DisableIRQ(USART1_IRQn);
}

void usart_2_enable(USART::Clock::Source a_clock_source, uint32_t a_irq_priority)
{
    assert(a_clock_source != USART::Clock::Source::unknown);

    constexpr uint32_t clock_source_lut[] = { 0, RCC_CCIPR_USART2SEL_0, RCC_CCIPR_USART2SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART2SEL, clock_source_lut[static_cast<uint32_t>(a_clock_source)]);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN);

    NVIC_SetPriority(USART2_IRQn, a_irq_priority);
    NVIC_EnableIRQ(USART2_IRQn);
}

void usart_2_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN);
    NVIC_DisableIRQ(USART2_IRQn);
}

void usart_3_enable(USART::Clock::Source a_clock_source, uint32_t a_irq_priority)
{
    constexpr uint32_t clock_source_lut[] = { 0, RCC_CCIPR_USART3SEL_0, RCC_CCIPR_USART3SEL_1 };
    set_flag(&(RCC->CCIPR), RCC_CCIPR_USART3SEL, clock_source_lut[static_cast<int32_t>(a_clock_source)]);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN);

    NVIC_SetPriority(USART3_IRQn, a_irq_priority);
    NVIC_EnableIRQ(USART3_IRQn);
}

void usart_3_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN);
    NVIC_DisableIRQ(USART3_IRQn);
}

Controller controllers[] = { { USART1, nullptr, nullptr, usart_1_enable, usart_1_disable },
                             { USART2, nullptr, nullptr, usart_2_enable, usart_2_disable },
                             { USART3, nullptr, nullptr, usart_3_enable, usart_3_disable } };

bool is_USART_ISR_error(uint32_t a_isr)
{
    return is_any_bit(a_isr, USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
}

USART::Bus_status_flag get_bus_status_flag_from_USART_ISR(uint32_t a_isr)
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

void clear_USART_ISR_errors(volatile uint32_t* a_p_icr)
{
    set_flag(a_p_icr, USART_ICR_PECF | USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
}

#ifdef CML_ASSERT
bool is_usart(USART::Id a_id)
{
    return nullptr != controllers[static_cast<uint32_t>(a_id)].p_usart_handle;
}

bool is_rs485(USART::Id a_id)
{
    return nullptr != controllers[static_cast<uint32_t>(a_id)].p_rs485_handle;
}
#endif // CML_ASSERT

USART_TypeDef* get_usart_ptr(USART::Id a_id)
{
    return controllers[static_cast<uint32_t>(a_id)].p_registers;
}

} // namespace

extern "C" {

static void interrupt_handler(uint32_t a_index)
{
    assert((nullptr != controllers[a_index].p_usart_handle && nullptr == controllers[a_index].p_rs485_handle) ||
           (nullptr == controllers[a_index].p_usart_handle && nullptr != controllers[a_index].p_rs485_handle));

    if (nullptr != controllers[a_index].p_usart_handle)
    {
        usart_interrupt_handler(controllers[a_index].p_usart_handle);
    }
    else if (nullptr != controllers[a_index].p_rs485_handle)
    {
        rs485_interrupt_handler(controllers[a_index].p_rs485_handle);
    }
}

void USART1_IRQHandler()
{
    interrupt_handler(0);
}

void USART2_IRQHandler()
{
    interrupt_handler(1);
}

void USART3_IRQHandler()
{
    interrupt_handler(2);
}

} // extern "C"

namespace soc {
namespace stm32l452xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void usart_interrupt_handler(USART* a_p_this)
{
    assert(nullptr != a_p_this);

    const uint32_t isr = get_usart_ptr(a_p_this->id)->ISR;
    const uint32_t cr1 = get_usart_ptr(a_p_this->id)->CR1;
    const uint32_t cr3 = get_usart_ptr(a_p_this->id)->CR3;

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == is_flag(isr, USART_ISR_TXE) && true == is_flag(cr1, USART_CR1_TXEIE))
        {
            a_p_this->transmit_callback.function(
                &(get_usart_ptr(a_p_this->id)->TDR), false, a_p_this, a_p_this->transmit_callback.p_user_data);
        }

        if (true == is_flag(isr, USART_ISR_TC) && true == is_flag(cr1, USART_CR1_TCIE))
        {
            a_p_this->transmit_callback.function(nullptr, true, a_p_this, a_p_this->transmit_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == is_flag(isr, USART_ISR_RXNE) && true == is_flag(cr1, USART_CR1_RXNEIE))
        {
            a_p_this->receive_callback.function(
                get_usart_ptr(a_p_this->id)->RDR, false, a_p_this, a_p_this->receive_callback.p_user_data);
        }
        else if (true == is_flag(isr, USART_ISR_IDLE) && true == is_flag(cr1, USART_CR1_IDLEIE))
        {
            set_flag(&(get_usart_ptr(a_p_this->id)->ICR), USART_ICR_IDLECF);
            a_p_this->receive_callback.function(0x0u, true, a_p_this, a_p_this->receive_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->bus_status_callback.function && true == is_flag(cr3, USART_CR3_EIE) &&
        true == is_flag(cr1, USART_CR1_PEIE))
    {
        USART::Bus_status_flag status = get_bus_status_flag_from_USART_ISR(isr);

        if (status != USART::Bus_status_flag::ok)
        {
            a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            clear_USART_ISR_errors(&(get_usart_ptr(a_p_this->id)->ICR));
        }
    }
}

void rs485_interrupt_handler(RS485* a_p_this)
{
    assert(nullptr != a_p_this);

    const uint32_t isr = get_usart_ptr(a_p_this->id)->ISR;
    const uint32_t cr1 = get_usart_ptr(a_p_this->id)->CR1;
    const uint32_t cr3 = get_usart_ptr(a_p_this->id)->CR3;

    if (nullptr != a_p_this->transmit_callback.function)
    {
        if (true == is_flag(isr, USART_ISR_TXE) && true == is_flag(cr1, USART_CR1_TXEIE))
        {
            a_p_this->transmit_callback.function(
                &(get_usart_ptr(a_p_this->id)->TDR), false, a_p_this, a_p_this->transmit_callback.p_user_data);
        }

        if (true == is_flag(isr, USART_ISR_TC) && true == is_flag(cr1, USART_CR1_TCIE))
        {
            a_p_this->transmit_callback.function(nullptr, true, a_p_this, a_p_this->transmit_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->receive_callback.function)
    {
        if (true == is_flag(isr, USART_ISR_RXNE) && true == is_flag(cr1, USART_CR1_RXNEIE))
        {
            volatile uint16_t rdr = get_usart_ptr(a_p_this->id)->RDR;

            if (false == is_flag(rdr, 0x100))
            {
                a_p_this->receive_callback.function(rdr, false, a_p_this, a_p_this->receive_callback.p_user_data);
            }
        }
        else if (true == is_flag(isr, USART_ISR_IDLE) && true == is_flag(cr1, USART_CR1_IDLEIE))
        {
            set_flag(&(get_usart_ptr(a_p_this->id)->ICR), USART_ICR_IDLECF);
            a_p_this->receive_callback.function(0x0u, true, a_p_this, a_p_this->receive_callback.p_user_data);
        }
    }

    if (nullptr != a_p_this->bus_status_callback.function && true == is_flag(cr3, USART_CR3_EIE) &&
        true == is_flag(cr1, USART_CR1_PEIE))
    {
        USART::Bus_status_flag status = get_bus_status_flag_from_USART_ISR(isr);

        if (status != USART::Bus_status_flag::ok)
        {
            a_p_this->bus_status_callback.function(status, a_p_this, a_p_this->bus_status_callback.p_user_data);
            clear_USART_ISR_errors(&(get_usart_ptr(a_p_this->id)->ICR));
        }
    }
}

bool USART::enable(const Config& a_config,
                   const Frame_format& a_frame_format,
                   const Clock& a_clock,
                   uint32_t a_irq_priority,
                   time::tick a_timeout_ms)
{
    assert(false == this->is_enabled());
    assert(false == is_usart(this->id) && false == is_rs485(this->id));

    assert(0 != a_config.baud_rate);
    assert(Flow_control_flag::unknown != a_config.flow_control);
    assert(Stop_bits::unknown != a_config.stop_bits);
    assert(Sampling_method::unknown != a_config.sampling_method);

    assert(Parity::unknown != a_frame_format.parity);
    assert(Word_length::unknown != a_frame_format.word_length);

    assert(Clock::Source::unknown != a_clock.source);
    assert(0 != a_clock.frequency_hz);
    assert(a_timeout_ms > 0);

    time::tick start = counter::get();

    controllers[static_cast<uint32_t>(this->id)].p_usart_handle = this;
    controllers[static_cast<uint32_t>(this->id)].enable(a_clock.source, a_irq_priority);

    switch (a_config.oversampling)
    {
        case Oversampling::_16: {
            get_usart_ptr(this->id)->BRR = a_clock.frequency_hz / a_config.baud_rate;
        }
        break;

        case Oversampling::_8: {
            const uint32_t usartdiv      = 2 * a_clock.frequency_hz / a_config.baud_rate;
            get_usart_ptr(this->id)->BRR = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::unknown: {
            assert(a_config.oversampling != Oversampling::unknown);
        }
        break;
    }

    get_usart_ptr(this->id)->CR2 = static_cast<uint32_t>(a_config.stop_bits);
    get_usart_ptr(this->id)->CR3 =
        static_cast<uint32_t>(a_config.flow_control) | static_cast<uint32_t>(a_config.sampling_method);

    get_usart_ptr(this->id)->CR1 = static_cast<uint32_t>(a_config.oversampling) | static_cast<uint32_t>(a_config.mode) |
                                   static_cast<uint32_t>(a_frame_format.parity) |
                                   static_cast<uint32_t>(a_frame_format.word_length) | USART_CR1_UE;

    this->baud_rate    = a_config.baud_rate;
    this->clock        = a_clock;
    this->frame_format = a_frame_format;

    uint32_t wait_flag = (true == is_flag(get_usart_ptr(this->id)->CR1, USART_CR1_RE) ? USART_ISR_REACK : 0) |
                         (true == is_flag(get_usart_ptr(this->id)->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0);

    return wait::until(&(get_usart_ptr(this->id)->ISR), wait_flag, false, start, a_timeout_ms);
}

void USART::disable()
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    get_usart_ptr(this->id)->CR1 = 0;
    get_usart_ptr(this->id)->CR2 = 0;
    get_usart_ptr(this->id)->CR3 = 0;

    controllers[static_cast<uint32_t>(this->id)].disable();
    controllers[static_cast<uint32_t>(this->id)].p_usart_handle = nullptr;
}

USART::Result USART::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_TCCF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_TC) && false == error)
    {
        if (true == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_TXE) && words < a_data_size_in_words)
        {
            if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
            {
                get_usart_ptr(this->id)->TDR = (static_cast<const uint16_t*>(a_p_data)[words++]) & 0x1FFu;
            }
            else
            {
                get_usart_ptr(this->id)->TDR = (static_cast<const uint8_t*>(a_p_data)[words++]) & 0xFFu;
            }
        }

        error = is_USART_ISR_error(get_usart_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR(get_usart_ptr(this->id)->ISR);
        clear_USART_ISR_errors(&(get_usart_ptr(this->id)->ICR));
    }

    return { bus_status, words };
}

USART::Result
USART::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, time::tick a_timeout_ms)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);
    assert(a_timeout_ms > 0);

    time::tick start = counter::get();

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_TCCF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_TC) && false == error &&
           a_timeout_ms < time::diff(counter::get(), start))
    {
        if (true == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_TXE) && words < a_data_size_in_words)
        {
            if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
            {
                get_usart_ptr(this->id)->TDR = (static_cast<const uint16_t*>(a_p_data)[words++]) & 0x1FFu;
            }
            else
            {
                get_usart_ptr(this->id)->TDR = (static_cast<const uint8_t*>(a_p_data)[words++]) & 0xFFu;
            }
        }

        error = is_USART_ISR_error(get_usart_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR(get_usart_ptr(this->id)->ISR);
        clear_USART_ISR_errors(&(get_usart_ptr(this->id)->ICR));
    }

    return { bus_status, words };
}

USART::Result USART::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_IDLECF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_IDLE) && false == error)
    {
        if (true == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_RXNE))
        {
            if (words < a_data_size_in_words)
            {
                if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
                {
                    static_cast<uint16_t*>(a_p_data)[words++] = (get_usart_ptr(this->id)->RDR & 0x1FFu);
                }
                else
                {
                    static_cast<uint8_t*>(a_p_data)[words++] = (get_usart_ptr(this->id)->RDR & 0xFFu);
                }
            }
            else
            {
                set_flag(&(get_usart_ptr(this->id)->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_USART_ISR_error(get_usart_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR(get_usart_ptr(this->id)->ISR);
        clear_USART_ISR_errors(&(get_usart_ptr(this->id)->ICR));
    }

    return { bus_status, words };
}

USART::Result USART::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, time::tick a_timeout_ms)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);
    assert(a_timeout_ms > 0);

    time::tick start = counter::get();

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_IDLECF);

    uint32_t ret               = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_IDLE) && false == error &&
           a_timeout_ms >= time::diff(counter::get(), start))
    {
        if (true == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_RXNE))
        {
            if (ret < a_data_size_in_words)
            {
                if (Parity::none == this->frame_format.parity && Word_length::_9_bit == this->frame_format.word_length)
                {
                    static_cast<uint16_t*>(a_p_data)[ret++] = (get_usart_ptr(this->id)->RDR & 0x1FFu);
                }
                else
                {
                    static_cast<uint8_t*>(a_p_data)[ret++] = (get_usart_ptr(this->id)->RDR & 0xFFu);
                }
            }
            else
            {
                set_flag(&(get_usart_ptr(this->id)->RQR), USART_RQR_RXFRQ);
                ret++;
            }
        }

        error = is_USART_ISR_error(get_usart_ptr(this->id)->ISR);
    }

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR(get_usart_ptr(this->id)->ISR);
        clear_USART_ISR_errors(&(get_usart_ptr(this->id)->ICR));
    }

    return { bus_status, ret };
}

void USART::register_transmit_callback(const Transmit_callback& a_callback)
{
    assert(true == this->is_enabled());
    assert(false == this->is_transmit_callback());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_TCCF);
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);
}

void USART::register_receive_callback(const Receive_callback& a_callback)
{
    assert(true == this->is_enabled());
    assert(false == this->is_receive_callback());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_IDLECF);
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
}

void USART::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    assert(true == this->is_enabled());
    assert(false == this->is_bus_status_callback());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;

    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_PEIE);
    set_flag(&(get_usart_ptr(this->id)->CR3), USART_CR3_EIE);
}

void USART::unregister_transmit_callback()
{
    assert(true == this->is_enabled());
    assert(true == this->is_transmit_callback());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    Interrupt_guard guard;

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);

    this->transmit_callback = { nullptr, nullptr };
}

void USART::unregister_receive_callback()
{
    assert(true == this->is_enabled());
    assert(true == this->is_receive_callback());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    Interrupt_guard guard;

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);

    this->receive_callback = { nullptr, nullptr };
}

void USART::unregister_bus_status_callback()
{
    assert(true == this->is_enabled());
    assert(true == this->is_bus_status_callback());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    Interrupt_guard guard;

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_PEIE);
    clear_flag(&(get_usart_ptr(this->id)->CR3), USART_CR3_EIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void USART::set_baud_rate(uint32_t a_baud_rate)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(0 != a_baud_rate);

    const Oversampling oversampling = this->get_oversampling();

    switch (oversampling)
    {
        case Oversampling::_8: {
            const uint32_t usartdiv      = 2 * this->clock.frequency_hz / a_baud_rate;
            get_usart_ptr(this->id)->BRR = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::_16: {
            get_usart_ptr(this->id)->BRR = this->clock.frequency_hz / a_baud_rate;
        }
        break;

        case Oversampling::unknown: {
            assert(Oversampling::unknown != oversampling);
        }
        break;
    }
}

void USART::set_oversampling(Oversampling a_oversampling)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(Oversampling::unknown != a_oversampling);

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
    set_flag(&(get_usart_ptr(this->id)->CR1), static_cast<uint32_t>(a_oversampling));
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
}

void USART::set_stop_bits(Stop_bits a_stop_bits)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(Stop_bits::unknown != a_stop_bits);

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
    set_flag(&(get_usart_ptr(this->id)->CR2), static_cast<uint32_t>(a_stop_bits));
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
}

void USART::set_flow_control(Flow_control_flag a_flow_control)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(Flow_control_flag::unknown != a_flow_control);

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
    set_flag(&(get_usart_ptr(this->id)->CR3), static_cast<uint32_t>(a_flow_control));
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
}

void USART::set_sampling_method(Sampling_method a_sampling_method)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(Sampling_method::unknown != a_sampling_method);

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
    set_flag(&(get_usart_ptr(this->id)->CR3), USART_CR3_ONEBIT, static_cast<uint32_t>(a_sampling_method));
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
}

void USART::set_frame_format(const Frame_format& a_frame_format)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(USART::Word_length::unknown != a_frame_format.word_length);
    assert(USART::Parity::unknown != a_frame_format.parity);

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
    set_flag(&(get_usart_ptr(this->id)->CR1),
             USART_CR1_PCE | USART_CR1_M,
             static_cast<uint32_t>(a_frame_format.parity) | static_cast<uint32_t>(a_frame_format.word_length) |
                 USART_CR1_UE);

    this->frame_format = a_frame_format;
}

bool USART::set_mode(Mode_flag a_mode, time::tick a_timeout_ms)
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    assert(Mode_flag::unknown != a_mode);
    assert(a_timeout_ms > 0);

    time::tick start = counter::get();

    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_TE | USART_CR1_RE, static_cast<uint32_t>(a_mode));

    uint32_t wait_flag = (true == is_flag(get_usart_ptr(this->id)->CR1, USART_CR1_RE) ? USART_ISR_REACK : 0) |
                         (true == is_flag(get_usart_ptr(this->id)->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0);

    return wait::until(&(get_usart_ptr(this->id)->ISR), wait_flag, false, start, a_timeout_ms);
}

USART::Oversampling USART::get_oversampling() const
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    return static_cast<Oversampling>(get_flag(get_usart_ptr(this->id)->CR1, static_cast<uint32_t>(USART_CR1_OVER8)));
}

USART::Stop_bits USART::get_stop_bits() const
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    return static_cast<Stop_bits>(get_flag(get_usart_ptr(this->id)->CR2, USART_CR2_STOP));
}

USART::Flow_control_flag USART::get_flow_control() const
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    return static_cast<Flow_control_flag>(get_flag(get_usart_ptr(this->id)->CR3, USART_CR3_RTSE | USART_CR3_CTSE));
}

USART::Sampling_method USART::get_sampling_method() const
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    return static_cast<Sampling_method>(get_flag(get_usart_ptr(this->id)->CR3, USART_CR3_ONEBIT));
}

USART::Mode_flag USART::get_mode() const
{
    assert(true == this->is_enabled());
    assert(true == is_usart(this->id) && false == is_rs485(this->id));

    return static_cast<Mode_flag>(get_flag(get_usart_ptr(this->id)->CR1, USART_CR1_TE | USART_CR1_RE));
}

bool USART::is_enabled() const
{
    return is_flag(controllers[static_cast<uint32_t>(this->id)].p_registers->CR1, USART_CR1_UE);
}

bool RS485::enable(const Config& a_config,
                   const USART::Clock& a_clock,
                   pin::Out* a_p_flow_control_pin,
                   uint32_t a_irq_priority,
                   time::tick a_timeout)
{
    assert(false == this->is_enabled());
    assert(false == is_usart(this->id) && false == is_rs485(this->id));

    assert(nullptr != a_p_flow_control_pin);
    assert(0 != a_config.baud_rate);
    assert(Stop_bits::unknown != a_config.stop_bits);

    assert(USART::Clock::Source::unknown != a_clock.source);
    assert(0 != a_clock.frequency_hz);
    assert(a_timeout > 0);

    time::tick start = counter::get();

    controllers[static_cast<uint32_t>(this->id)].p_rs485_handle = this;
    controllers[static_cast<uint32_t>(this->id)].enable(a_clock.source, a_irq_priority);

    switch (a_config.oversampling)
    {
        case Oversampling::_16: {
            get_usart_ptr(this->id)->BRR = a_clock.frequency_hz / a_config.baud_rate;
        }
        break;

        case Oversampling::_8: {
            const uint32_t usartdiv      = 2 * a_clock.frequency_hz / a_config.baud_rate;
            get_usart_ptr(this->id)->BRR = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::unknown: {
            assert(a_config.oversampling != Oversampling::unknown);
        }
        break;
    }

    get_usart_ptr(this->id)->CR3 = USART_CR3_ONEBIT;
    get_usart_ptr(this->id)->CR2 =
        static_cast<uint32_t>(a_config.stop_bits) | (a_config.address << USART_CR2_ADD_Pos) | USART_CR2_ADDM7;

    get_usart_ptr(this->id)->CR1 = static_cast<uint32_t>(a_config.oversampling) | USART_CR1_M0 | USART_CR1_UE |
                                   USART_CR1_TE | USART_CR1_RE | USART_CR1_MME | USART_CR1_WAKE;

    get_usart_ptr(this->id)->RQR = USART_RQR_MMRQ;

    this->p_flow_control_pin = a_p_flow_control_pin;
    this->baud_rate          = a_config.baud_rate;
    this->clock              = a_clock;

    bool ret = wait::until(
        &(get_usart_ptr(this->id)->ISR), USART_ISR_TEACK | USART_ISR_REACK | USART_ISR_RWU, false, start, a_timeout);

    if (true == ret)
    {
        this->p_flow_control_pin->set_level(pin::Level::low);
    }

    return ret;
}

void RS485::disable()
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    get_usart_ptr(this->id)->CR1 = 0;
    get_usart_ptr(this->id)->CR2 = 0;
    get_usart_ptr(this->id)->CR3 = 0;
    get_usart_ptr(this->id)->RQR = 0;

    controllers[static_cast<uint32_t>(this->id)].disable();
    controllers[static_cast<uint32_t>(this->id)].p_rs485_handle = nullptr;

    this->p_flow_control_pin = nullptr;
}

RS485::Result RS485::transmit_bytes_polling(uint8_t a_address, const void* a_p_data, uint32_t a_data_size_in_words)
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    assert(a_address <= 0x7F);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_TCCF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    this->p_flow_control_pin->set_level(pin::Level::high);

    while (false == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_TC) && false == error)
    {
        if (true == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_TXE))
        {
            if (words == 0)
            {
                get_usart_ptr(this->id)->TDR = (static_cast<uint16_t>(a_address) | static_cast<uint16_t>(0x100u));
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                get_usart_ptr(this->id)->TDR = (static_cast<const uint8_t*>(a_p_data)[words - 1]);
                words++;
            }
        }

        error = is_USART_ISR_error(get_usart_ptr(this->id)->ISR);
    }

    this->p_flow_control_pin->set_level(pin::Level::low);

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR(get_usart_ptr(this->id)->ISR);
        clear_USART_ISR_errors(&(get_usart_ptr(this->id)->ICR));
    }

    return { bus_status, words };
}

RS485::Result RS485::transmit_bytes_polling(uint8_t a_address,
                                            const void* a_p_data,
                                            uint32_t a_data_size_in_words,
                                            time::tick a_timeout_ms)
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    assert(a_address <= 0x7F);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);
    assert(a_timeout_ms > 0);

    time::tick start = counter::get();

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_TCCF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    this->p_flow_control_pin->set_level(pin::Level::high);

    while (false == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_TC) && false == error &&
           a_timeout_ms < time::diff(counter::get(), start))
    {
        if (true == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_TXE))
        {
            if (words == 0)
            {
                get_usart_ptr(this->id)->TDR = (static_cast<uint16_t>(a_address) | static_cast<uint16_t>(0x100u));
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                get_usart_ptr(this->id)->TDR = (static_cast<const uint8_t*>(a_p_data)[words - 1]);
                words++;
            }
        }

        error = is_USART_ISR_error(get_usart_ptr(this->id)->ISR);
    }

    this->p_flow_control_pin->set_level(pin::Level::low);

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR(get_usart_ptr(this->id)->ISR);
        clear_USART_ISR_errors(&(get_usart_ptr(this->id)->ICR));
    }

    return { bus_status, words };
}

RS485::Result RS485::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words)
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_IDLECF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_IDLE) && false == error)
    {
        if (true == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_RXNE))
        {
            if (0 == words)
            {
                set_flag(&(get_usart_ptr(this->id)->RQR), USART_RQR_RXFRQ);
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                static_cast<uint8_t*>(a_p_data)[words - 1] = (get_usart_ptr(this->id)->RDR & 0xFFu);
                words++;
            }
            else
            {
                set_flag(&(get_usart_ptr(this->id)->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_USART_ISR_error(get_usart_ptr(this->id)->ISR);
    }

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_CMCF);

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR(get_usart_ptr(this->id)->ISR);
        clear_USART_ISR_errors(&(get_usart_ptr(this->id)->ICR));
    }

    return { bus_status, words };
}

RS485::Result RS485::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, time::tick a_timeout_ms)
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);
    assert(a_timeout_ms > 0);

    time::tick start = counter::get();

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_IDLECF);

    uint32_t words             = 0;
    bool error                 = false;
    Bus_status_flag bus_status = Bus_status_flag::ok;

    while (false == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_IDLE) && false == error &&
           a_timeout_ms >= time::diff(counter::get(), start))
    {
        if (true == is_flag(get_usart_ptr(this->id)->ISR, USART_ISR_RXNE))
        {
            if (0 == words)
            {
                set_flag(&(get_usart_ptr(this->id)->RQR), USART_RQR_RXFRQ);
                words++;
            }
            else if (words < a_data_size_in_words + 1)
            {
                static_cast<uint8_t*>(a_p_data)[words - 1] = (get_usart_ptr(this->id)->RDR & 0xFFu);
                words++;
            }
            else
            {
                set_flag(&(get_usart_ptr(this->id)->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }

        error = is_USART_ISR_error(get_usart_ptr(this->id)->ISR);
    }

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_CMCF);

    if (true == error)
    {
        bus_status = get_bus_status_flag_from_USART_ISR(get_usart_ptr(this->id)->ISR);
        clear_USART_ISR_errors(&(get_usart_ptr(this->id)->ICR));
    }

    return { bus_status, words };
}

void RS485::register_transmit_callback(const Transmit_callback& a_callback)
{
    assert(true == this->is_enabled());
    assert(false == this->is_transmit_callback());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_TCCF);
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);

    this->p_flow_control_pin->set_level(pin::Level::high);
}

void RS485::register_receive_callback(const Receive_callback& a_callback)
{
    assert(true == this->is_enabled());
    assert(false == this->is_receive_callback());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_IDLECF);
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);
}

void RS485::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    assert(true == this->is_enabled());
    assert(false == this->is_bus_status_callback());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;

    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_PEIE);
    set_flag(&(get_usart_ptr(this->id)->CR3), USART_CR3_EIE);
}

void RS485::unregister_transmit_callback()
{
    assert(true == this->is_enabled());
    assert(true == this->is_transmit_callback());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    Interrupt_guard guard;

    this->p_flow_control_pin->set_level(pin::Level::low);

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_TCIE | USART_CR1_TXEIE);

    this->transmit_callback = { nullptr, nullptr };
}

void RS485::unregister_receive_callback()
{
    assert(true == this->is_enabled());
    assert(true == this->is_receive_callback());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    Interrupt_guard guard;

    set_flag(&(get_usart_ptr(this->id)->ICR), USART_ICR_CMCF);
    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_RXNEIE);

    this->receive_callback = { nullptr, nullptr };
}

void RS485::unregister_bus_status_callback()
{
    assert(true == this->is_enabled());
    assert(true == this->is_bus_status_callback());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    Interrupt_guard guard;

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_PEIE);
    clear_flag(&(get_usart_ptr(this->id)->CR3), USART_CR3_EIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void RS485::set_baud_rate(uint32_t a_baud_rate)
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    assert(0 != a_baud_rate);

    const Oversampling oversampling = this->get_oversampling();

    switch (oversampling)
    {
        case Oversampling::_8: {
            const uint32_t usartdiv      = 2 * this->clock.frequency_hz / a_baud_rate;
            get_usart_ptr(this->id)->BRR = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
        }
        break;

        case Oversampling::_16: {
            get_usart_ptr(this->id)->BRR = this->clock.frequency_hz / a_baud_rate;
        }
        break;

        case Oversampling::unknown: {
            assert(Oversampling::unknown != oversampling);
        }
        break;
    }
}

void RS485::set_oversampling(Oversampling a_oversampling)
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    assert(Oversampling::unknown != a_oversampling);

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
    set_flag(&(get_usart_ptr(this->id)->CR1), static_cast<uint32_t>(a_oversampling));
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
}

void RS485::set_stop_bits(Stop_bits a_stop_bits)
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    assert(Stop_bits::unknown != a_stop_bits);

    clear_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
    set_flag(&(get_usart_ptr(this->id)->CR2), static_cast<uint32_t>(a_stop_bits));
    set_flag(&(get_usart_ptr(this->id)->CR1), USART_CR1_UE);
}

bool RS485::is_enabled() const
{
    return is_flag(controllers[static_cast<uint32_t>(this->id)].p_registers->CR1, USART_CR1_UE);
}

RS485::Oversampling RS485::get_oversampling() const
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    return static_cast<Oversampling>(get_flag(get_usart_ptr(this->id)->CR1, static_cast<uint32_t>(USART_CR1_OVER8)));
}

RS485::Stop_bits RS485::get_stop_bits() const
{
    assert(true == this->is_enabled());
    assert(false == is_usart(this->id) && true == is_rs485(this->id));

    return static_cast<Stop_bits>(get_flag(get_usart_ptr(this->id)->CR2, USART_CR2_STOP));
}

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx