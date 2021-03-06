/*
 *   Name: SPI.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/peripherals/SPI.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l011xx::peripherals;

enum class Direction_flag : uint32_t
{
    transmit,
    receive
};

constexpr Direction_flag operator|(Direction_flag a_f1, Direction_flag a_f2)
{
    return static_cast<Direction_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr Direction_flag operator&(Direction_flag a_f1, Direction_flag a_f2)
{
    return static_cast<Direction_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr Direction_flag operator|=(Direction_flag& a_f1, Direction_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

void spi_1_enable(uint32_t a_irq_priority)
{
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);

    NVIC_SetPriority(SPI1_IRQn, a_irq_priority);
    NVIC_EnableIRQ(SPI1_IRQn);
}

void spi_1_disable()
{
    bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);
    NVIC_DisableIRQ(SPI1_IRQn);
}

SPI_master* p_master = nullptr;
SPI_slave* p_slave   = nullptr;

bool is_SPI_SR_error(Direction_flag a_direction)
{
    return bit::is_any(
        SPI1->SR, SPI_SR_FRE | (Direction_flag::receive == a_direction ? SPI_SR_OVR : 0) | SPI_SR_MODF | SPI_SR_CRCERR);
}

SPI_base::Bus_flag get_bus_flag_from_SPI_SR()
{
    SPI_base::Bus_flag ret = SPI_base::Bus_flag::ok;
    uint32_t sr            = SPI1->SR;

    if (true == bit_flag::is(sr, SPI_SR_OVR))
    {
        ret |= SPI_base::Bus_flag::overrun;
    }

    if (true == bit_flag::is(sr, SPI_SR_FRE))
    {
        ret |= SPI_base::Bus_flag::frame_error;
    }

    if (true == bit_flag::is(sr, SPI_SR_MODF))
    {
        ret |= SPI_base::Bus_flag::mode_fault;
    }

    if (true == bit_flag::is(sr, SPI_SR_CRCERR))
    {
        ret |= SPI_base::Bus_flag::crc_error;
    }

    return ret;
}

void clear_SPI_SR_overrun()
{
    volatile uint32_t t1 = SPI1->DR;
    volatile uint32_t t2 = SPI1->SR;

    unused(t1);
    unused(t2);
}

void clear_SPI_SR_errors()
{
    const uint32_t sr = SPI1->SR;

    if (true == bit_flag::is(sr, SPI_SR_OVR))
    {
        volatile uint32_t t1 = SPI1->DR;
        volatile uint32_t t2 = SPI1->SR;

        unused(t1);
        unused(t2);
    }

    if (true == bit_flag::is(sr, SPI_SR_MODF))
    {
        volatile uint32_t t1 = SPI1->SR;
        unused(t1);

        bit_flag::set(&(SPI1->CR1), SPI_CR1_SPE);
    }

    if (true == bit_flag::is(sr, SPI_SR_FRE))
    {
        volatile uint32_t t1 = SPI1->SR;
        unused(t1);
    }

    if (true == bit_flag::is(sr, SPI_SR_CRCERR))
    {
        bit::clear(&(SPI1->SR), SPI_SR_CRCERR_Pos);
    }
}

} // namespace

extern "C" {

void SPI1_IRQHandler()
{
    cml_assert(nullptr != p_master || nullptr != p_slave);

    if (nullptr != p_master)
    {
        spi_interrupt_handler(p_master);
    }
    else if (nullptr != p_slave)
    {
        spi_interrupt_handler(p_slave);
    }
}

} // extern "C"

namespace soc {
namespace stm32l011xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void spi_interrupt_handler(SPI_base* a_p_this)
{
    const uint32_t sr  = SPI1->SR;
    const uint32_t cr1 = SPI1->CR1;
    const uint32_t cr2 = SPI1->CR2;

    if (true == bit::is_any(sr, SPI_SR_FRE | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR))
    {
        if (nullptr != a_p_this->bus_status_callback.function)
        {
            a_p_this->bus_status_callback.function(
                get_bus_flag_from_SPI_SR(), a_p_this, a_p_this->transmit_callback.p_user_data);
        }

        clear_SPI_SR_errors();
    }

    if (nullptr != a_p_this->transmit_callback.function || nullptr != a_p_this->transmit_receive_callback.transmit)
    {
        if (true == bit_flag::is(sr, SPI_SR_TXE) && true == bit_flag::is(cr2, SPI_CR2_TXEIE))
        {
            if (nullptr != a_p_this->transmit_callback.function)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr1, SPI_CR1_DFF)) ==
                    SPI_master::Frame_format::Word_length::_16)
                {
                    a_p_this->transmit_callback.function(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR)),
                                                         false,
                                                         a_p_this,
                                                         a_p_this->transmit_callback.p_user_data);
                }
                else
                {
                    a_p_this->transmit_callback.function(
                        reinterpret_cast<volatile uint16_t*>(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))),
                        false,
                        a_p_this,
                        a_p_this->transmit_callback.p_user_data);
                }
            }

            if (nullptr != a_p_this->transmit_receive_callback.transmit)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr1, SPI_CR1_DFF)) ==
                    SPI_master::Frame_format::Word_length::_16)
                {
                    a_p_this->transmit_receive_callback.transmit(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR)),
                                                                 false,
                                                                 a_p_this,
                                                                 a_p_this->transmit_receive_callback.p_user_data);
                }
                else
                {
                    a_p_this->transmit_receive_callback.transmit(
                        reinterpret_cast<volatile uint16_t*>(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))),
                        false,
                        a_p_this,
                        a_p_this->transmit_receive_callback.p_user_data);
                }
            }

            if (false == bit_flag::is(SPI1->SR, SPI_SR_BSY))
            {
                if (nullptr != a_p_this->transmit_callback.function)
                {
                    a_p_this->transmit_callback.function(
                        nullptr, true, a_p_this, a_p_this->transmit_callback.p_user_data);
                }

                if (nullptr != a_p_this->transmit_receive_callback.transmit)
                {
                    a_p_this->transmit_receive_callback.transmit(
                        nullptr, true, a_p_this, a_p_this->transmit_receive_callback.p_user_data);
                }
            }
        }
    }

    if (nullptr != a_p_this->receive_callback.function || nullptr != a_p_this->transmit_receive_callback.receive)
    {
        if (true == bit_flag::is(sr, SPI_SR_RXNE) && true == bit_flag::is(cr2, SPI_CR2_RXNEIE))
        {
            if (nullptr != a_p_this->receive_callback.function)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr1, SPI_CR1_DFF)) ==
                    SPI_master::Frame_format::Word_length::_16)
                {
                    a_p_this->receive_callback.function((*reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))),
                                                        a_p_this,
                                                        a_p_this->receive_callback.p_user_data);
                }
                else
                {
                    a_p_this->receive_callback.function((*reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))),
                                                        a_p_this,
                                                        a_p_this->receive_callback.p_user_data);
                }
            }

            if (nullptr != a_p_this->transmit_receive_callback.receive)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR1_DFF)) ==
                    SPI_master::Frame_format::Word_length::_16)
                {
                    a_p_this->transmit_receive_callback.receive((*reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))),
                                                                a_p_this,
                                                                a_p_this->transmit_receive_callback.p_user_data);
                }
                else
                {
                    a_p_this->transmit_receive_callback.receive((*reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))),
                                                                a_p_this,
                                                                a_p_this->transmit_receive_callback.p_user_data);
                }
            }
        }
    }
}

void SPI_base::register_transmit_callback(const Transmit_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->transmit_callback = a_callback;

    bit_flag::set(&(SPI1->CR2), SPI_CR2_TXEIE);
}

void SPI_base::register_receive_callback(const Receive_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    bit_flag::set(&(SPI1->CR2), SPI_CR2_RXNEIE);
}

void SPI_base::register_transmit_receive_callaback(const Transmit_receive_callback& a_callback)
{
    cml_assert(nullptr != a_callback.transmit);
    cml_assert(nullptr != a_callback.receive);

    Interrupt_guard guard;

    this->transmit_receive_callback = a_callback;

    bit_flag::set(&(SPI1->CR2), SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
}

void SPI_base::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;

    bit_flag::set(&(SPI1->CR2), SPI_CR2_ERRIE);
}

void SPI_base::unregister_transmit_callback()
{
    cml_assert(nullptr != this->transmit_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(SPI1->CR2), SPI_CR2_TXEIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void SPI_base::unregister_receive_callback()
{
    cml_assert(nullptr != this->receive_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(SPI1->CR2), SPI_CR2_RXNEIE);

    this->receive_callback = { nullptr, nullptr };
}

void SPI_base::unregister_transmit_receive_callback()
{
    cml_assert(nullptr != this->transmit_receive_callback.receive &&
               nullptr != this->transmit_receive_callback.transmit);

    Interrupt_guard guard;

    bit_flag::clear(&(SPI1->CR2), SPI_CR2_RXNEIE | SPI_CR2_TXEIE);

    this->transmit_receive_callback = { nullptr, nullptr, nullptr };
}

void SPI_base::unregister_bus_status_callback()
{
    cml_assert(nullptr != this->bus_status_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(SPI1->CR2), SPI_CR2_ERRIE);

    this->bus_status_callback = { nullptr, nullptr };
}

SPI_base::Frame_format SPI_base::get_frame_format() const
{
    return { true == bit_flag::is(SPI1->CR1, SPI_CR1_CPOL) ? Frame_format::Polarity::high : Frame_format::Polarity::low,
             true == bit_flag::is(SPI1->CR1, SPI_CR1_CPHA) ? Frame_format::Phase::second_edge :
                                                             Frame_format::Phase::first_edge,
             static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)),
             true == bit_flag::is(SPI1->CR1, SPI_CR1_LSBFIRST) ? Frame_format::Bit_significance::least :
                                                                 Frame_format::Bit_significance::most };
}
SPI_base::Clock_source SPI_base::get_clock_source() const
{
    return { Clock_source::Type::pclkx, static_cast<Clock_source::Prescaler>(bit_flag::get(SPI1->CR1, SPI_CR1_BR)) };
}

void SPI_master::enable(const Config& a_config,
                        const Frame_format& a_frame_format,
                        const Clock_source& a_clock_source,
                        uint32_t a_irq_priority)
{
    cml_assert(Config::Wiring::unknown != a_config.wiring);
    cml_assert(Config::Crc::unknown != a_config.crc);
    cml_assert(Config::NSS_management::unknown != a_config.nss_management);
    cml_assert(Frame_format::Bit_significance::unknown != a_frame_format.bit_significance);
    cml_assert(Frame_format::Phase::unknown != a_frame_format.phase);
    cml_assert(Frame_format::Polarity::unknown != a_frame_format.polarity);
    cml_assert(Frame_format::Word_length::unknown != a_frame_format.word_length);
    cml_assert(Clock_source::Prescaler::unknown != a_clock_source.prescaler);
    cml_assert(Clock_source::Type::unknown != a_clock_source.type);

    // not implemented Config::Wiring::half_duplex and Config::Wiring::Simplex
    cml_assert(Config::Wiring::full_duplex == a_config.wiring);

    cml_assert(nullptr == p_slave && nullptr == p_master);

    spi_1_enable(a_irq_priority);
    p_slave  = nullptr;
    p_master = this;

    SPI1->CR2 = static_cast<uint32_t>(a_frame_format.word_length) | SPI_CR2_SSOE |
                (Config::NSS_management::hardware == a_config.nss_management ? SPI_CR2_SSOE : 0x0u);

    SPI1->CR1 =
        SPI_CR1_MSTR | static_cast<uint32_t>(a_config.wiring) | static_cast<uint32_t>(a_frame_format.phase) |
        static_cast<uint32_t>(a_frame_format.polarity) | static_cast<uint32_t>(a_frame_format.bit_significance) |
        static_cast<uint32_t>(a_clock_source.prescaler) | (Config::Crc::enable == a_config.crc ? SPI_CR1_CRCEN : 0x0u) |
        (Config::NSS_management::software == a_config.nss_management ? SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE : 0x0u);
}

void SPI_master::disable()
{
    if (false == bit_flag::is(SPI1->CR1, SPI_CR1_BIDIMODE) && true == bit_flag::is(SPI1->CR1, SPI_CR1_RXONLY))
    {
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
        wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);
    }
    else
    {
        wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
    }

    SPI1->CR2 = 0;
    SPI1->CR1 = 0;

    spi_1_disable();
    p_master = nullptr;
}

SPI_master::Result
SPI_master::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(SPI1->CR1), SPI_CR1_SPE);
    }

    bool error        = false;
    bool busy         = true;
    uint32_t words    = 0;
    Bus_flag bus_flag = Bus_flag::ok;

    while (true == busy && false == error)
    {
        if (true == bit_flag::is(SPI1->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) = static_cast<const uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) = static_cast<const uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_SPI_SR_error(Direction_flag::transmit);
        busy  = bit_flag::is(SPI1->SR, SPI_SR_BSY);
    }

    clear_SPI_SR_overrun();

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, words };
}

SPI_master::Result SPI_master::transmit_bytes_polling(const void* a_p_data,
                                                      uint32_t a_data_size_in_words,
                                                      uint32_t a_timeout,
                                                      GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    uint32_t start = system_timer::get();

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(SPI1->CR1), SPI_CR1_SPE);
    }

    bool error        = false;
    bool busy         = true;
    uint32_t words    = 0;
    Bus_flag bus_flag = Bus_flag::ok;

    while (true == busy && false == error && a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(SPI1->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) = static_cast<const uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) = static_cast<const uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_SPI_SR_error(Direction_flag::transmit);
        busy  = bit_flag::is(SPI1->SR, SPI_SR_BSY);
    }

    clear_SPI_SR_overrun();

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, words };
}

SPI_master::Result
SPI_master::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(SPI1->CR1), SPI_CR1_SPE);
    }

    bool error           = false;
    bool transmit_enable = true;
    uint32_t words       = 0;
    Bus_flag bus_flag    = Bus_flag::ok;

    while (false == error && words < a_data_size_in_words)
    {
        if (true == transmit_enable && true == bit_flag::is(SPI1->SR, SPI_SR_TXE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) = static_cast<uint16_t>(0xFFFFu);
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) = static_cast<uint8_t>(0xFFu);
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(SPI1->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                static_cast<uint16_t*>(a_p_data)[words++] = static_cast<uint16_t>(SPI1->DR);
            }
            else
            {
                static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(SPI1->DR);
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(Direction_flag::receive);
    }

    wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, words };
}

SPI_master::Result SPI_master::receive_bytes_polling(void* a_p_data,
                                                     uint32_t a_data_size_in_words,
                                                     uint32_t a_timeout,
                                                     GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    uint32_t start = system_timer::get();

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(SPI1->CR1), SPI_CR1_SPE);
    }

    bool error           = false;
    bool transmit_enable = true;
    uint32_t words       = 0;
    Bus_flag bus_flag    = Bus_flag::ok;

    while (false == error && words < a_data_size_in_words &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == transmit_enable && true == bit_flag::is(SPI1->SR, SPI_SR_TXE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) = static_cast<uint16_t>(0xFFFFu);
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) = static_cast<uint8_t>(0xFFu);
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(SPI1->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                static_cast<uint16_t*>(a_p_data)[words++] = static_cast<uint16_t>(SPI1->DR);
            }
            else
            {
                static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(SPI1->DR);
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(Direction_flag::receive);
    }

    wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, words };
}

SPI_master::Result SPI_master::transmit_receive_bytes_polling(const void* a_p_tx_data,
                                                              void* a_p_rx_data,
                                                              uint32_t a_tx_rx_data_size_in_words,
                                                              GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_tx_data);
    cml_assert(nullptr != a_p_rx_data);
    cml_assert(a_tx_rx_data_size_in_words > 0);

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(SPI1->CR1), SPI_CR1_SPE);
    }

    bool error           = false;
    bool transmit_enable = true;
    Bus_flag bus_flag    = Bus_flag::ok;

    uint32_t tx_idx = 0;
    uint32_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words)
    {
        if (true == transmit_enable && true == bit_flag::is(SPI1->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) =
                    static_cast<const uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) =
                    static_cast<const uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(SPI1->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                uint16_t t = static_cast<uint16_t>(SPI1->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                uint8_t t = static_cast<uint8_t>(SPI1->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, rx_idx };
}

SPI_master::Result SPI_master::transmit_receive_bytes_polling(const void* a_p_tx_data,
                                                              void* a_p_rx_data,
                                                              uint32_t a_tx_rx_data_size_in_words,
                                                              uint32_t a_timeout,
                                                              GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_tx_data);
    cml_assert(nullptr != a_p_rx_data);
    cml_assert(a_tx_rx_data_size_in_words > 0);

    uint32_t start = system_timer::get();

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(SPI1->CR1), SPI_CR1_SPE);
    }

    bool error           = false;
    bool transmit_enable = true;
    Bus_flag bus_flag    = Bus_flag::ok;

    uint32_t tx_idx = 0;
    uint32_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == transmit_enable && true == bit_flag::is(SPI1->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) >
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) =
                    static_cast<const uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) =
                    static_cast<const uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(SPI1->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                uint16_t t = static_cast<uint16_t>(SPI1->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                uint8_t t = static_cast<uint8_t>(SPI1->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(SPI1->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, rx_idx };
}

SPI_master::Config SPI_master::get_config() const
{
    return { true == bit_flag::is(SPI1->CR1, SPI_CR1_BIDIMODE) ?
                 Config::Wiring::half_duplex :
                 (true == bit_flag::is(SPI1->CR1, SPI_CR1_RXONLY) ? Config::Wiring::simplex :
                                                                    Config::Wiring::full_duplex),
             (true == bit_flag::is(SPI1->CR1, SPI_CR1_SSM | SPI_CR1_SSI) ? Config::NSS_management::software :
                                                                           Config::NSS_management::hardware),
             static_cast<Config::Crc>(bit_flag::get(SPI1->CR1, SPI_CR1_CRCEN)) };
}

void SPI_slave::enable(const Config& a_config,
                       const Frame_format& a_frame_format,
                       const Clock_source& a_clock_source,
                       uint32_t a_irq_priority)
{
    cml_assert(Config::Wiring::unknown != a_config.wiring);
    cml_assert(Config::Crc::unknown != a_config.crc);
    cml_assert(Frame_format::Bit_significance::unknown != a_frame_format.bit_significance);
    cml_assert(Frame_format::Phase::unknown != a_frame_format.phase);
    cml_assert(Frame_format::Polarity::unknown != a_frame_format.polarity);
    cml_assert(Frame_format::Word_length::unknown != a_frame_format.word_length);
    cml_assert(Clock_source::Prescaler::unknown != a_clock_source.prescaler);
    cml_assert(Clock_source::Type::unknown != a_clock_source.type);

    // not implemented Config::Wiring::half_duplex and Config::Wiring::Simplex
    cml_assert(Config::Wiring::full_duplex == a_config.wiring);

    cml_assert(nullptr == p_slave && nullptr == p_master);

    spi_1_enable(a_irq_priority);
    p_slave  = this;
    p_master = nullptr;

    SPI1->CR2 = static_cast<uint32_t>(a_frame_format.word_length) | SPI_CR2_SSOE;

    SPI1->CR1 = static_cast<uint32_t>(a_config.wiring) | static_cast<uint32_t>(a_frame_format.phase) |
                static_cast<uint32_t>(a_frame_format.polarity) |
                static_cast<uint32_t>(a_frame_format.bit_significance) |
                static_cast<uint32_t>(a_clock_source.prescaler) |
                (Config::Crc::enable == a_config.crc ? SPI_CR1_CRCEN : 0x0u) | SPI_CR1_SPE;
}

void SPI_slave::disable()
{
    if (false == bit_flag::is(SPI1->CR1, SPI_CR1_BIDIMODE) && true == bit_flag::is(SPI1->CR1, SPI_CR1_RXONLY))
    {
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
        wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);
    }
    else
    {
        wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);
        bit_flag::clear(&(SPI1->CR1), SPI_CR1_SPE);
    }

    SPI1->CR2 = 0;
    SPI1->CR1 = 0;

    spi_1_disable();
    p_master = nullptr;
}

SPI_slave::Result SPI_slave::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    bool error        = false;
    bool busy         = true;
    uint32_t words    = 0;
    Bus_flag bus_flag = Bus_flag::ok;

    while (true == busy && false == error)
    {
        if (true == bit_flag::is(SPI1->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) = static_cast<const uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) = static_cast<const uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_SPI_SR_error(Direction_flag::transmit);
        busy  = bit_flag::is(SPI1->SR, SPI_SR_BSY);
    }

    clear_SPI_SR_overrun();

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    return { bus_flag, words };
}

SPI_slave::Result
SPI_slave::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    uint32_t start = system_timer::get();

    bool error        = false;
    bool busy         = true;
    uint32_t words    = 0;
    Bus_flag bus_flag = Bus_flag::ok;

    while (true == busy && false == error && a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(SPI1->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) = static_cast<const uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) = static_cast<const uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_SPI_SR_error(Direction_flag::transmit);
        busy  = bit_flag::is(SPI1->SR, SPI_SR_BSY);
    }

    clear_SPI_SR_overrun();

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    return { bus_flag, words };
}

SPI_slave::Result SPI_slave::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    bool error        = false;
    uint32_t words    = 0;
    Bus_flag bus_flag = Bus_flag::ok;

    while (false == error && words < a_data_size_in_words)
    {
        if (true == bit_flag::is(SPI1->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                static_cast<uint16_t*>(a_p_data)[words++] = static_cast<uint16_t>(SPI1->DR);
            }
            else
            {
                static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(SPI1->DR);
            }
        }

        error = is_SPI_SR_error(Direction_flag::receive);
    }

    wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    return { bus_flag, words };
}

SPI_slave::Result SPI_slave::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    uint32_t start = system_timer::get();

    bool error        = false;
    uint32_t words    = 0;
    Bus_flag bus_flag = Bus_flag::ok;

    while (false == error && words < a_data_size_in_words &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(SPI1->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR2, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                static_cast<uint16_t*>(a_p_data)[words++] = static_cast<uint16_t>(SPI1->DR);
            }
            else
            {
                static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(SPI1->DR);
            }
        }

        error = is_SPI_SR_error(Direction_flag::receive);
    }

    wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    return { bus_flag, words };
}

SPI_slave::Result SPI_slave::transmit_receive_bytes_polling(const void* a_p_tx_data,
                                                            void* a_p_rx_data,
                                                            uint32_t a_tx_rx_data_size_in_words)
{
    cml_assert(nullptr != a_p_tx_data);
    cml_assert(nullptr != a_p_rx_data);
    cml_assert(a_tx_rx_data_size_in_words > 0);

    bool error           = false;
    bool transmit_enable = true;
    Bus_flag bus_flag    = Bus_flag::ok;

    uint32_t tx_idx = 0;
    uint32_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words)
    {
        if (true == transmit_enable && true == bit_flag::is(SPI1->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) =
                    static_cast<const uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) =
                    static_cast<const uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(SPI1->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                uint16_t t = static_cast<uint16_t>(SPI1->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                uint8_t t = static_cast<uint8_t>(SPI1->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    return { bus_flag, rx_idx };
}

SPI_slave::Result SPI_slave::transmit_receive_bytes_polling(const void* a_p_tx_data,
                                                            void* a_p_rx_data,
                                                            uint32_t a_tx_rx_data_size_in_words,
                                                            uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_tx_data);
    cml_assert(nullptr != a_p_rx_data);
    cml_assert(a_tx_rx_data_size_in_words > 0);

    uint32_t start = system_timer::get();

    bool error           = false;
    bool transmit_enable = true;
    Bus_flag bus_flag    = Bus_flag::ok;

    uint32_t tx_idx = 0;
    uint32_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == transmit_enable && true == bit_flag::is(SPI1->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(SPI1->DR))) =
                    static_cast<const uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(SPI1->DR))) =
                    static_cast<const uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(SPI1->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(SPI1->CR1, SPI_CR1_DFF)) ==
                Frame_format::Word_length::_16)
            {
                uint16_t t = static_cast<uint16_t>(SPI1->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                uint8_t t = static_cast<uint8_t>(SPI1->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::all_bits(&(SPI1->SR), SPI_SR_BSY, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR();

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors();
        }
    }

    return { bus_flag, rx_idx };
}

SPI_slave::Config SPI_slave::get_config() const
{
    return { true == bit_flag::is(SPI1->CR1, SPI_CR1_BIDIMODE) ?
                 Config::Wiring::half_duplex :
                 (true == bit_flag::is(SPI1->CR1, SPI_CR1_RXONLY) ? Config::Wiring::simplex :
                                                                    Config::Wiring::full_duplex),
             static_cast<Config::Crc>(bit_flag::get(SPI1->CR1, SPI_CR1_CRCEN)) };
}

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L452xx