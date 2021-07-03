/*
 *   Name: SPI.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/peripherals/SPI.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

// externals
#include <stm32l4xx.h>

// TODO: CML - SPI half-duplex and simplex implementation

namespace {

using namespace cml;
using namespace soc::m4::stm32l4::peripherals;

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

struct Controller
{
    SPI_TypeDef* p_registers        = nullptr;
    SPI_master* p_spi_master_handle = nullptr;
    SPI_slave* p_spi_slave_handle   = nullptr;
};

Controller controllers[]
{
    { SPI1, nullptr, nullptr },
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        { SPI2, nullptr, nullptr },
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    {
        SPI3, nullptr, nullptr
    }
#endif
};

SPI_TypeDef* get_spi_ptr(SPI_base::Id a_id)
{
    return controllers[static_cast<uint32_t>(a_id)].p_registers;
}

bool is_SPI_SR_error(SPI_base::Id a_id, Direction_flag a_direction)
{
    return bit::is_any(get_spi_ptr(a_id)->SR,
                       SPI_SR_FRE | (Direction_flag::receive == a_direction ? SPI_SR_OVR : 0) | SPI_SR_MODF |
                           SPI_SR_CRCERR);
}

SPI_base::Bus_flag get_bus_flag_from_SPI_SR(SPI_base::Id a_id)
{
    SPI_base::Bus_flag ret = SPI_base::Bus_flag::ok;
    uint32_t sr            = get_spi_ptr(a_id)->SR;

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

void clear_SPI_SR_overrun(SPI_base::Id a_id)
{
    volatile uint32_t t1 = get_spi_ptr(a_id)->DR;
    volatile uint32_t t2 = get_spi_ptr(a_id)->SR;

    unused(t1);
    unused(t2);
}

void clear_SPI_SR_errors(SPI_base::Id a_id)
{
    const uint32_t sr = get_spi_ptr(a_id)->SR;

    if (true == bit_flag::is(sr, SPI_SR_OVR))
    {
        volatile uint32_t t1 = get_spi_ptr(a_id)->DR;
        volatile uint32_t t2 = get_spi_ptr(a_id)->SR;

        unused(t1);
        unused(t2);
    }

    if (true == bit_flag::is(sr, SPI_SR_MODF))
    {
        volatile uint32_t t1 = get_spi_ptr(a_id)->SR;
        unused(t1);

        bit_flag::set(&(get_spi_ptr(a_id)->CR1), SPI_CR1_SPE);
    }

    if (true == bit_flag::is(sr, SPI_SR_FRE))
    {
        volatile uint32_t t1 = get_spi_ptr(a_id)->SR;
        unused(t1);
    }

    if (true == bit_flag::is(sr, SPI_SR_CRCERR))
    {
        bit::clear(&(get_spi_ptr(a_id)->SR), SPI_SR_CRCERR_Pos);
    }
}

} // namespace

extern "C" {

void interupt_handler(uint32_t a_controller_index)
{
    cml_assert(nullptr != controllers[a_controller_index].p_spi_master_handle ||
               nullptr != controllers[a_controller_index].p_spi_slave_handle);

    if (nullptr != controllers[a_controller_index].p_spi_master_handle)
    {
        spi_interrupt_handler(controllers[a_controller_index].p_spi_master_handle);
    }
    else if (nullptr != controllers[a_controller_index].p_spi_slave_handle)
    {
        spi_interrupt_handler(controllers[a_controller_index].p_spi_slave_handle);
    }
}

void SPI1_IRQHandler()
{
    interupt_handler(0);
}

void SPI2_IRQHandler()
{
    interupt_handler(1);
}

void SPI3_IRQHandler()
{
    interupt_handler(2);
}


} // extern "C"

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void spi_interrupt_handler(SPI_base* a_p_this)
{
    const uint32_t sr  = get_spi_ptr(a_p_this->id)->SR;
    const uint32_t cr2 = get_spi_ptr(a_p_this->id)->CR2;

    if (true == bit::is_any(sr, SPI_SR_FRE | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR))
    {
        if (nullptr != a_p_this->bus_status_callback.function)
        {
            a_p_this->bus_status_callback.function(
                get_bus_flag_from_SPI_SR(a_p_this->id), a_p_this, a_p_this->transmit_callback.p_user_data);
        }

        clear_SPI_SR_errors(a_p_this->id);
    }

    if (nullptr != a_p_this->transmit_callback.function || nullptr != a_p_this->transmit_receive_callback.transmit)
    {
        if (true == bit_flag::is(sr, SPI_SR_TXE) && true == bit_flag::is(cr2, SPI_CR2_TXEIE))
        {
            if (nullptr != a_p_this->transmit_callback.function)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
                    SPI_master::Frame_format::Word_length::_8)
                {
                    a_p_this->transmit_callback.function(
                        reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(a_p_this->id)->DR)),
                        false,
                        a_p_this,
                        a_p_this->transmit_callback.p_user_data);
                }
                else
                {
                    a_p_this->transmit_callback.function(
                        reinterpret_cast<volatile uint16_t*>(
                            reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(a_p_this->id)->DR))),
                        false,
                        a_p_this,
                        a_p_this->transmit_callback.p_user_data);
                }
            }

            if (nullptr != a_p_this->transmit_receive_callback.transmit)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
                    SPI_master::Frame_format::Word_length::_8)
                {
                    a_p_this->transmit_receive_callback.transmit(
                        reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(a_p_this->id)->DR)),
                        false,
                        a_p_this,
                        a_p_this->transmit_receive_callback.p_user_data);
                }
                else
                {
                    a_p_this->transmit_receive_callback.transmit(
                        reinterpret_cast<volatile uint16_t*>(
                            reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(a_p_this->id)->DR))),
                        false,
                        a_p_this,
                        a_p_this->transmit_receive_callback.p_user_data);
                }
            }

            if (false == bit_flag::is(get_spi_ptr(a_p_this->id)->SR, SPI_SR_BSY))
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
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
                    SPI_master::Frame_format::Word_length::_8)
                {
                    a_p_this->receive_callback.function(
                        (*reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(a_p_this->id)->DR))),
                        a_p_this,
                        a_p_this->receive_callback.p_user_data);
                }
                else
                {
                    a_p_this->receive_callback.function(
                        (*reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(a_p_this->id)->DR))),
                        a_p_this,
                        a_p_this->receive_callback.p_user_data);
                }
            }

            if (nullptr != a_p_this->transmit_receive_callback.receive)
            {
                if (static_cast<SPI_master::Frame_format::Word_length>(bit_flag::get(cr2, SPI_CR2_DS)) >
                    SPI_master::Frame_format::Word_length::_8)
                {
                    a_p_this->transmit_receive_callback.receive(
                        (*reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(a_p_this->id)->DR))),
                        a_p_this,
                        a_p_this->transmit_receive_callback.p_user_data);
                }
                else
                {
                    a_p_this->transmit_receive_callback.receive(
                        (*reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(a_p_this->id)->DR))),
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

    bit_flag::set(&(get_spi_ptr(this->id)->CR2), SPI_CR2_TXEIE);
}

void SPI_base::register_receive_callback(const Receive_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->receive_callback = a_callback;

    bit_flag::set(&(get_spi_ptr(this->id)->CR2), SPI_CR2_RXNEIE);
}

void SPI_base::register_transmit_receive_callaback(const Transmit_receive_callback& a_callback)
{
    cml_assert(nullptr != a_callback.transmit);
    cml_assert(nullptr != a_callback.receive);

    Interrupt_guard guard;

    this->transmit_receive_callback = a_callback;

    bit_flag::set(&(get_spi_ptr(this->id)->CR2), SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
}

void SPI_base::register_bus_status_callback(const Bus_status_callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->bus_status_callback = a_callback;

    bit_flag::set(&(get_spi_ptr(this->id)->CR2), SPI_CR2_ERRIE);
}

void SPI_base::unregister_transmit_callback()
{
    cml_assert(nullptr != this->transmit_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(get_spi_ptr(this->id)->CR2), SPI_CR2_TXEIE);

    this->bus_status_callback = { nullptr, nullptr };
}

void SPI_base::unregister_receive_callback()
{
    cml_assert(nullptr != this->receive_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(get_spi_ptr(this->id)->CR2), SPI_CR2_RXNEIE);

    this->receive_callback = { nullptr, nullptr };
}

void SPI_base::unregister_transmit_receive_callback()
{
    cml_assert(nullptr != this->transmit_receive_callback.receive &&
               nullptr != this->transmit_receive_callback.transmit);

    Interrupt_guard guard;

    bit_flag::clear(&(get_spi_ptr(this->id)->CR2), SPI_CR2_RXNEIE | SPI_CR2_TXEIE);

    this->transmit_receive_callback = { nullptr, nullptr, nullptr };
}

void SPI_base::unregister_bus_status_callback()
{
    cml_assert(nullptr != this->bus_status_callback.function);

    Interrupt_guard guard;

    bit_flag::clear(&(get_spi_ptr(this->id)->CR2), SPI_CR2_ERRIE);

    this->bus_status_callback = { nullptr, nullptr };
}

SPI_base::Frame_format SPI_base::get_frame_format() const
{
    return { true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_CPOL) ? Frame_format::Polarity::high :
                                                                              Frame_format::Polarity::low,
             true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_CPHA) ? Frame_format::Phase::second_edge :
                                                                              Frame_format::Phase::first_edge,
             static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)),
             true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_LSBFIRST) ?
                 Frame_format::Bit_significance::least :
                 Frame_format::Bit_significance::most };
}

void SPI_master::enable(const Config& a_config, const Frame_format& a_frame_format, uint32_t a_irq_priority)
{
    cml_assert(various::get_enum_incorrect_value<Config::Wiring>() != a_config.wiring);
    cml_assert(various::get_enum_incorrect_value<Config::Crc>() != a_config.crc);
    cml_assert(various::get_enum_incorrect_value<Config::NSS_management>() != a_config.nss_management);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Bit_significance>() != a_frame_format.bit_significance);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Phase>() != a_frame_format.phase);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Polarity>() != a_frame_format.polarity);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Word_length>() != a_frame_format.word_length);
    cml_assert(various::get_enum_incorrect_value<Config::Clock_prescaler>() != a_config.clock_prescaler);

    // not implemented Config::Wiring::half_duplex and Config::Wiring::Simplex
    cml_assert(Config::Wiring::full_duplex == a_config.wiring);

    cml_assert(nullptr == controllers[static_cast<uint32_t>(this->id)].p_spi_slave_handle &&
               nullptr == controllers[static_cast<uint32_t>(this->id)].p_spi_master_handle);

    controllers[static_cast<uint32_t>(this->id)].p_spi_slave_handle  = nullptr;
    controllers[static_cast<uint32_t>(this->id)].p_spi_master_handle = this;

    get_spi_ptr(this->id)->CR2 = static_cast<uint32_t>(a_frame_format.word_length) | SPI_CR2_SSOE |
                                 (a_frame_format.word_length <= Frame_format::Word_length::_8 ? SPI_CR2_FRXTH : 0x0u) |
                                 (Config::NSS_management::hardware == a_config.nss_management ? SPI_CR2_SSOE : 0x0u);

    get_spi_ptr(this->id)->CR1 =
        SPI_CR1_MSTR | static_cast<uint32_t>(a_config.wiring) | static_cast<uint32_t>(a_frame_format.phase) |
        static_cast<uint32_t>(a_frame_format.polarity) | static_cast<uint32_t>(a_frame_format.bit_significance) |
        static_cast<uint32_t>(a_config.clock_prescaler) | (Config::Crc::enable == a_config.crc ? SPI_CR1_CRCEN : 0x0u) |
        (Config::NSS_management::software == a_config.nss_management ? SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE : 0x0u);

    switch (this->id)
    {
        case Id::_1: {
            NVIC_SetPriority(IRQn_Type::SPI1_IRQn, a_irq_priority);
            NVIC_EnableIRQ(IRQn_Type::SPI1_IRQn);
        }
        break;
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Id::_2: {
            NVIC_SetPriority(IRQn_Type::SPI2_IRQn, a_irq_priority);
            NVIC_EnableIRQ(IRQn_Type::SPI2_IRQn);
        }
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Id::_3: {
            NVIC_SetPriority(IRQn_Type::SPI3_IRQn, a_irq_priority);
            NVIC_EnableIRQ(IRQn_Type::SPI3_IRQn);
        }
        break;
#endif
    }
}

void SPI_master::disable()
{
    switch (this->id)
    {
        case Id::_1: {
            NVIC_DisableIRQ(IRQn_Type::SPI1_IRQn);
        }
        break;
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Id::_2: {
            NVIC_DisableIRQ(IRQn_Type::SPI2_IRQn);
        }
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Id::_3: {
            NVIC_DisableIRQ(IRQn_Type::SPI3_IRQn);
        }
        break;
#endif
    }

    if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_BIDIMODE) &&
        true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_RXONLY))
    {
        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
        wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);

        while (true == bit::is_any(get_spi_ptr(this->id)->SR, SPI_SR_FRLVL))
        {
            volatile uint32_t dr = get_spi_ptr(this->id)->DR;
            unused(dr);
        }
    }
    else
    {
        wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);
        wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);

        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);

        while (true == bit::is_any(get_spi_ptr(this->id)->SR, SPI_SR_FRLVL))
        {
            volatile uint32_t dr = get_spi_ptr(this->id)->DR;
            unused(dr);
        }
    }

    get_spi_ptr(this->id)->CR2 = 0;
    get_spi_ptr(this->id)->CR1 = 0;

    controllers[static_cast<uint32_t>(this->id)].p_spi_master_handle = nullptr;
}

SPI_master::Result
SPI_master::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
    }

    bool error        = false;
    bool busy         = true;
    uint32_t words    = 0;
    Bus_flag bus_flag = Bus_flag::ok;

    while (true == busy && false == error)
    {
        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_SPI_SR_error(this->id, Direction_flag::transmit);
        busy  = bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_BSY);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);

    clear_SPI_SR_overrun(this->id);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
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

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
    }

    bool error        = false;
    bool busy         = true;
    uint32_t words    = 0;
    Bus_flag bus_flag = Bus_flag::ok;

    while (true == busy && false == error && a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_SPI_SR_error(this->id, Direction_flag::transmit);
        busy  = bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_BSY);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);

    clear_SPI_SR_overrun(this->id);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, words };
}

SPI_master::Result
SPI_master::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size_in_words > 0);

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
    }

    bool error           = false;
    bool transmit_enable = true;
    uint32_t words       = 0;
    Bus_flag bus_flag    = Bus_flag::ok;

    while (false == error && words < a_data_size_in_words)
    {
        if (true == transmit_enable && true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) = static_cast<uint16_t>(0xFFFFu);
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) = static_cast<uint8_t>(0xFFu);
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                static_cast<uint16_t*>(a_p_data)[words++] = static_cast<uint16_t>(get_spi_ptr(this->id)->DR);
            }
            else
            {
                static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(get_spi_ptr(this->id)->DR);
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(this->id, Direction_flag::receive);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);
    wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);
    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
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

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
    }

    bool error           = false;
    bool transmit_enable = true;
    uint32_t words       = 0;
    Bus_flag bus_flag    = Bus_flag::ok;

    while (false == error && words < a_data_size_in_words &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == transmit_enable && true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) = static_cast<uint16_t>(0xFFFFu);
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) = static_cast<uint8_t>(0xFFu);
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                static_cast<uint16_t*>(a_p_data)[words++] = static_cast<uint16_t>(get_spi_ptr(this->id)->DR);
            }
            else
            {
                static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(get_spi_ptr(this->id)->DR);
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(this->id, Direction_flag::receive);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);
    wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);
    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
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

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
    }

    bool error           = false;
    bool transmit_enable = true;
    Bus_flag bus_flag    = Bus_flag::ok;

    uint32_t tx_idx = 0;
    uint32_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words)
    {
        if (true == transmit_enable && true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                uint16_t t = static_cast<uint16_t>(get_spi_ptr(this->id)->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                uint8_t t = static_cast<uint8_t>(get_spi_ptr(this->id)->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(this->id, Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);
    wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);
    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
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

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::low);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::set(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
    }

    bool error           = false;
    bool transmit_enable = true;
    Bus_flag bus_flag    = Bus_flag::ok;

    uint32_t tx_idx = 0;
    uint32_t rx_idx = 0;

    while (false == error && rx_idx < a_tx_rx_data_size_in_words &&
           a_timeout >= various::time_diff(system_timer::get(), start))
    {
        if (true == transmit_enable && true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                uint16_t t = static_cast<uint16_t>(get_spi_ptr(this->id)->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                uint8_t t = static_cast<uint8_t>(get_spi_ptr(this->id)->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(this->id, Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);
    wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);
    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
        }
    }

    if (nullptr != a_p_nss && true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        a_p_nss->set_level(GPIO::Level::high);
    }
    else if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM))
    {
        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
    }

    return { bus_flag, rx_idx };
}

SPI_master::Config SPI_master::get_config() const
{
    return { static_cast<Config::Clock_prescaler>(bit_flag::get(get_spi_ptr(this->id)->CR1, SPI_CR1_BR_Msk)),
             true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_BIDIMODE) ?
                 Config::Wiring::half_duplex :
                 (true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_RXONLY) ? Config::Wiring::simplex :
                                                                                     Config::Wiring::full_duplex),
             (true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_SSM | SPI_CR1_SSI) ?
                  Config::NSS_management::software :
                  Config::NSS_management::hardware),
             static_cast<Config::Crc>(bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_CRCEN)) };
}

void SPI_slave::enable(const Config& a_config, const Frame_format& a_frame_format, uint32_t a_irq_priority)
{
    cml_assert(various::get_enum_incorrect_value<Config::Wiring>() != a_config.wiring);
    cml_assert(various::get_enum_incorrect_value<Config::Crc>() != a_config.crc);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Bit_significance>() != a_frame_format.bit_significance);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Phase>() != a_frame_format.phase);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Polarity>() != a_frame_format.polarity);
    cml_assert(various::get_enum_incorrect_value<Frame_format::Word_length>() != a_frame_format.word_length);
    cml_assert(various::get_enum_incorrect_value<Config::Clock_prescaler>() != a_config.clock_prescaler);

    // not implemented Config::Wiring::half_duplex and Config::Wiring::Simplex
    cml_assert(Config::Wiring::full_duplex == a_config.wiring);

    cml_assert(nullptr == controllers[static_cast<uint32_t>(this->id)].p_spi_slave_handle &&
               nullptr == controllers[static_cast<uint32_t>(this->id)].p_spi_master_handle);

    controllers[static_cast<uint32_t>(this->id)].p_spi_slave_handle  = this;
    controllers[static_cast<uint32_t>(this->id)].p_spi_master_handle = nullptr;

    get_spi_ptr(this->id)->CR2 = static_cast<uint32_t>(a_frame_format.word_length) | SPI_CR2_SSOE |
                                 (a_frame_format.word_length <= Frame_format::Word_length::_8 ? SPI_CR2_FRXTH : 0x0u);

    get_spi_ptr(this->id)->CR1 = static_cast<uint32_t>(a_config.wiring) | static_cast<uint32_t>(a_frame_format.phase) |
                                 static_cast<uint32_t>(a_frame_format.polarity) |
                                 static_cast<uint32_t>(a_frame_format.bit_significance) |
                                 static_cast<uint32_t>(a_config.clock_prescaler) |
                                 (Config::Crc::enable == a_config.crc ? SPI_CR1_CRCEN : 0x0u) | SPI_CR1_SPE;
}

void SPI_slave::disable()
{
    if (false == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_BIDIMODE) &&
        true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_RXONLY))
    {
        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);
        wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);

        while (true == bit::is_any(get_spi_ptr(this->id)->SR, SPI_SR_FRLVL))
        {
            volatile uint32_t dr = get_spi_ptr(this->id)->DR;
            unused(dr);
        }
    }
    else
    {
        wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);
        wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);

        bit_flag::clear(&(get_spi_ptr(this->id)->CR1), SPI_CR1_SPE);

        while (true == bit::is_any(get_spi_ptr(this->id)->SR, SPI_SR_FRLVL))
        {
            volatile uint32_t dr = get_spi_ptr(this->id)->DR;
            unused(dr);
        }
    }

    get_spi_ptr(this->id)->CR2 = 0;
    get_spi_ptr(this->id)->CR1 = 0;

    controllers[static_cast<uint32_t>(this->id)].p_spi_master_handle = nullptr;
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
        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_SPI_SR_error(this->id, Direction_flag::transmit);
        busy  = bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_BSY);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);

    clear_SPI_SR_overrun(this->id);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
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
        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint16_t*>(a_p_data)[words++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint8_t*>(a_p_data)[words++];
            }
        }

        error = is_SPI_SR_error(this->id, Direction_flag::transmit);
        busy  = bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_BSY);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);

    clear_SPI_SR_overrun(this->id);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
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
        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                static_cast<uint16_t*>(a_p_data)[words++] = static_cast<uint16_t>(get_spi_ptr(this->id)->DR);
            }
            else
            {
                static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(get_spi_ptr(this->id)->DR);
            }
        }

        error = is_SPI_SR_error(this->id, Direction_flag::receive);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);
    wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);
    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
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
        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                static_cast<uint16_t*>(a_p_data)[words++] = static_cast<uint16_t>(get_spi_ptr(this->id)->DR);
            }
            else
            {
                static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(get_spi_ptr(this->id)->DR);
            }
        }

        error = is_SPI_SR_error(this->id, Direction_flag::receive);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);
    wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);
    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
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
        if (true == transmit_enable && true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                uint16_t t = static_cast<uint16_t>(get_spi_ptr(this->id)->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                uint8_t t = static_cast<uint8_t>(get_spi_ptr(this->id)->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(this->id, Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);
    wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);
    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
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
        if (true == transmit_enable && true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) &&
            tx_idx < a_tx_rx_data_size_in_words)
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint16_t*>(a_p_tx_data)[tx_idx++];
            }
            else
            {
                *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) =
                    static_cast<const uint8_t*>(a_p_tx_data)[tx_idx++];
            }

            transmit_enable = false;
        }

        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            if (static_cast<Frame_format::Word_length>(bit_flag::get(get_spi_ptr(this->id)->CR2, SPI_CR2_DS)) >
                Frame_format::Word_length::_8)
            {
                uint16_t t = static_cast<uint16_t>(get_spi_ptr(this->id)->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint16_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }
            else
            {
                uint8_t t = static_cast<uint8_t>(get_spi_ptr(this->id)->DR);

                if (rx_idx < a_tx_rx_data_size_in_words)
                {
                    static_cast<uint8_t*>(a_p_rx_data)[rx_idx++] = t;
                }
            }

            transmit_enable = true;
        }

        error = is_SPI_SR_error(this->id, Direction_flag::transmit | Direction_flag::receive);
    }

    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FTLVL, true);
    wait_until::all_bits(&(get_spi_ptr(this->id)->SR), SPI_SR_BSY, true);
    wait_until::any_bit(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, true);

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
        }
    }

    return { bus_flag, rx_idx };
}

SPI_slave::Config SPI_slave::get_config() const
{
    return { static_cast<Config::Clock_prescaler>(bit_flag::get(get_spi_ptr(this->id)->CR1, SPI_CR1_BR_Msk)),
             true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_BIDIMODE) ?
                 Config::Wiring::half_duplex :
                 (true == bit_flag::is(get_spi_ptr(this->id)->CR1, SPI_CR1_RXONLY) ? Config::Wiring::simplex :
                                                                                     Config::Wiring::full_duplex),
             static_cast<Config::Crc>(bit_flag::get(get_spi_ptr(this->id)->CR1, SPI_CR1_CRCEN)) };
}

} // namespace peripherals
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace soc::m4::stm32l4::peripherals;

void rcc<SPI_base>::enable(SPI_base::Id a_id, bool a_enable_in_lp)
{
    switch (a_id)
    {
        case SPI_base::Id::_1: {
            bit::set(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN_Pos);

            if (true == a_enable_in_lp)
            {
                bit::set(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN_Pos);
            }
        }
        break;

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case SPI_base::Id::_2: {
            bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN_Pos);

            if (true == a_enable_in_lp)
            {
                bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI2SMEN_Pos);
            }
        }
        break;
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case SPI_base::Id::_3: {
            bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI3EN_Pos);

            if (true == a_enable_in_lp)
            {
                bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI3SMEN_Pos);
            }
        }
        break;
#endif
    }
}
void rcc<SPI_base>::disable(SPI_base::Id a_id)
{
    switch (a_id)
    {
        case SPI_base::Id::_1: {
            bit::clear(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN_Pos);
            bit::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN_Pos);
        }
        break;

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case SPI_base::Id::_2: {
            bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN_Pos);
            bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI2SMEN_Pos);
        }
        break;
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case SPI_base::Id::_3: {
            bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI3EN_Pos);
            bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI3SMEN_Pos);
        }
        break;
#endif
    }
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4