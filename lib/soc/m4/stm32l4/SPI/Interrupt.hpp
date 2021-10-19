#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/IRQ.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>
#include <soc/m4/stm32l4/SPI/SPI.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class SPI_interrupt : private cml::Non_copyable
{
public:
    using Transmit_callback         = SPI_base::Transmit_callback;
    using Receive_callback          = SPI_base::Receive_callback;
    using Transmit_receive_callback = SPI_base::Transmit_receive_callback;

    enum class Bus_flag : std::uint32_t
    {
        ok          = 0x0u,
        overrun     = 0x1u,
        crc_error   = 0x2u,
        frame_error = 0x4u,
        mode_fault  = 0x8u,
    };

    struct Bus_status_callback
    {
        using Function    = void (*)(Bus_flag a_flag, SPI_base* a_p_this, void* a_p_user_data);
        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    SPI_interrupt(SPI_base* a_p_SPI, Handle<SPI1_BASE>)
        : p_SPI(a_p_SPI)
        , irqn(IRQn_Type::SPI1_IRQn)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    SPI_interrupt(SPI_base* a_p_SPI, Handle<SPI2_BASE>)
        : p_SPI(a_p_SPI)
        , irqn(IRQn_Type::SPI2_IRQn)
    {
    }
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    SPI_interrupt(SPI_base* a_p_SPI, Handle<SPI3_BASE>)
        : p_SPI(a_p_SPI)
        , irqn(IRQn_Type::SPI3_IRQn)
    {
    }
#endif

    void enable(const IRQ& a_irq);
    void disable();

    void register_callback(const Transmit_callback& a_callback);
    void register_callback(const Receive_callback& a_callback);
    void register_callback(const Transmit_receive_callback& a_callback);
    void register_callback(const Bus_status_callback& a_callback);

protected:
    SPI_base* p_SPI;
    IRQn_Type irqn;

    Transmit_callback transmit_callback;
    Receive_callback receive_callback;
    Transmit_receive_callback transmit_receive_callback;
    Bus_status_callback bus_status_callback;

protected:
    friend void spi_interrupt_handler(SPI_interrupt* a_p_this);
};

template<> class Interrupt<SPI_master> : public SPI_interrupt
{
public:
    Interrupt(SPI_master* a_p_SPI, Handle<SPI1_BASE>)
        : SPI_interrupt(a_p_SPI, SPI_base::id::_1)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    Interrupt(SPI_master* a_p_SPI, Handle<SPI2_BASE>)
        : SPI_interrupt(a_p_SPI, SPI_base::id::_2)
    {
    }
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    Interrupt(SPI_master* a_p_SPI, Handle<SPI3_BASE>)
        : SPI_interrupt(a_p_SPI, SPI_base::id::_3)
    {
    }
#endif
};

template<> class Interrupt<SPI_slave> : public SPI_interrupt
{
public:
    Interrupt(SPI_slave* a_p_SPI, Handle<SPI1_BASE>)
        : SPI_interrupt(a_p_SPI, SPI_base::id::_1)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    Interrupt(SPI_slave* a_p_SPI, Handle<SPI2_BASE>)
        : SPI_interrupt(a_p_SPI, SPI_base::id::_2)
    {
    }
#endif
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    Interrupt(SPI_slave* a_p_SPI, Handle<SPI3_BASE>)
        : SPI_interrupt(a_p_SPI, SPI_base::id::_3)
    {
    }
#endif
};

constexpr SPI_interrupt::Bus_flag operator|(SPI_interrupt::Bus_flag a_f1, SPI_interrupt::Bus_flag a_f2)
{
    return static_cast<SPI_interrupt::Bus_flag>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}
constexpr SPI_interrupt::Bus_flag operator&(SPI_interrupt::Bus_flag a_f1, SPI_interrupt::Bus_flag a_f2)
{
    return static_cast<SPI_interrupt::Bus_flag>(static_cast<std::uint32_t>(a_f1) & static_cast<std::uint32_t>(a_f2));
}
constexpr SPI_interrupt::Bus_flag operator|=(SPI_interrupt::Bus_flag& a_f1, SPI_interrupt::Bus_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc