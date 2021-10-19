#pragma once

/*
 *   Name: Interrupt.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/Handle.hpp>
#include <soc/m4/stm32l4/I2C/I2C.hpp>
#include <soc/m4/stm32l4/IRQ.hpp>
#include <soc/m4/stm32l4/Interrupt.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Interrupt<I2C_master> : private cml::Non_copyable
{
public:
    enum class Bus_status_flag : std::uint32_t
    {
        ok               = 0x0,
        crc_error        = 0x1,
        buffer_error     = 0x2,
        arbitration_lost = 0x4,
        misplaced        = 0x8,
        nack             = 0x10,
    };

    struct Transmit_event_callback
    {
        using Function = void (*)(volatile std::uint32_t* a_p_data,
                                  bool a_stop,
                                  I2C_base* a_p_this,
                                  void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;

        std::uint8_t slave_address         = 0xFFu;
        std::uint32_t data_length_in_bytes = 0xFFu;
    };

    struct Receive_event_callback
    {
        using Function = void (*)(std::uint8_t a_data, bool a_stop, I2C_base* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;

        std::uint8_t slave_address         = 0xFFu;
        std::uint32_t data_length_in_bytes = 0xFFu;
    };

    struct Bus_status_callback
    {
        using Function = void (*)(Bus_status_flag a_bus_flag, I2C_base* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;

        std::uint8_t slave_address = 0xFF;
    };

public:
    Interrupt(I2C_master* a_p_I2C, Handle<I2C1_BASE>)
        : p_I2C(a_p_I2C)
        , irqn_ev(IRQn_Type::I2C1_EV_IRQn)
        , irqn_er(IRQn_Type::I2C1_ER_IRQn)
    {
    }
    Interrupt(I2C_master* a_p_I2C, Handle<I2C3_BASE>)
        : p_I2C(a_p_I2C)
        , irqn_ev(IRQn_Type::I2C3_EV_IRQn)
        , irqn_er(IRQn_Type::I2C3_ER_IRQn)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    Interrupt(I2C_master* a_p_I2C, Handle<I2C2_BASE>)
        : p_I2C(a_p_I2C)
        , irqn_ev(IRQn_Type::I2C2_EV_IRQn)
        , irqn_er(IRQn_Type::I2C2_ER_IRQn)
    {
    }
#endif
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    Interrupt(I2C_master* a_p_I2C, Handle<I2C4_BASE>)
        : p_I2C(a_p_I2C)
        , irqn_ev(IRQn_Type::I2C4_EV_IRQn)
        , irqn_er(IRQn_Type::I2C4_ER_IRQn)
    {
    }
#endif

    void enable(const IRQ& a_event_irq, const IRQ& a_bus_status_irq);
    void disable();

    void register_callback(const Transmit_event_callback& a_callback);
    void register_callback(const Receive_event_callback& a_callback);
    void register_callback(const Bus_status_callback& a_callback);

private:
    I2C_master* p_I2C;
    IRQn_Type irqn_ev;
    IRQn_Type irqn_er;

    Transmit_event_callback transmit_event_callback;
    Receive_event_callback receive_event_callback;
    Bus_status_callback bus_status_callback;

private:
    friend void i2c_master_event_interrupt_handler(Interrupt<I2C_master>* a_p_this);
    friend void i2c_master_bus_status_interrupt_handler(Interrupt<I2C_master>* a_p_this);
};

template<> class Interrupt<I2C_slave> : private cml::Non_copyable
{
public:
    enum class Bus_status_flag : std::uint32_t
    {
        ok               = 0x0,
        crc_error        = 0x1,
        buffer_error     = 0x2,
        arbitration_lost = 0x4,
        misplaced        = 0x8,
        nack             = 0x10,
    };

    struct Transmit_event_callback
    {
        using Function = void (*)(volatile std::uint32_t* a_p_data,
                                  bool a_stop,
                                  I2C_base* a_p_this,
                                  void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;

        std::uint32_t data_length_in_bytes = 0xFF;
    };

    struct Receive_event_callback
    {
        using Function = void (*)(uint8_t a_data, bool a_stop, I2C_base* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;

        std::uint32_t data_length_in_bytes = 0xFF;
    };

    struct Address_match_event_callback
    {
        using Function = void (*)(I2C_base* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_status_callback
    {
        using Function = void (*)(Bus_status_flag a_bus_flag, I2C_base* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    Interrupt(I2C_slave* a_p_I2C, Handle<I2C1_BASE>)
        : p_I2C(a_p_I2C)
        , irqn_ev(IRQn_Type::I2C1_EV_IRQn)
        , irqn_er(IRQn_Type::I2C1_ER_IRQn)
    {
    }
    Interrupt(I2C_slave* a_p_I2C, Handle<I2C3_BASE>)
        : p_I2C(a_p_I2C)
        , irqn_ev(IRQn_Type::I2C3_EV_IRQn)
        , irqn_er(IRQn_Type::I2C3_ER_IRQn)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    Interrupt(I2C_slave* a_p_I2C, Handle<I2C2_BASE>)
        : p_I2C(a_p_I2C)
        , irqn_ev(IRQn_Type::I2C2_EV_IRQn)
        , irqn_er(IRQn_Type::I2C2_ER_IRQn)
    {
    }
#endif
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    Interrupt(I2C_slave* a_p_I2C, Handle<I2C4_BASE>)
        : p_I2C(a_p_I2C)
        , irqn_ev(IRQn_Type::I2C4_EV_IRQn)
        , irqn_er(IRQn_Type::I2C4_ER_IRQn)
    {
    }
#endif

    void enable(const IRQ& a_event_irq, const IRQ& a_bus_status_irq);
    void disable();

    void register_callback(const Transmit_event_callback& a_callback);
    void register_callback(const Receive_event_callback& a_callback);
    void register_callback(const Address_match_event_callback& a_callback);
    void register_callback(const Bus_status_callback& a_callback);

private:
    I2C_slave* p_I2C;
    IRQn_Type irqn_ev;
    IRQn_Type irqn_er;

    Transmit_event_callback transmit_event_callback;
    Receive_event_callback receive_event_callback;
    Address_match_event_callback address_match_event_callback;
    Bus_status_callback bus_status_callback;

private:
    friend void i2c_slave_event_interrupt_handler(Interrupt<I2C_slave>* a_p_this);
    friend void i2c_slave_bus_status_interrupt_handler(Interrupt<I2C_slave>* a_p_this);
};

constexpr Interrupt<I2C_master>::Bus_status_flag operator|(Interrupt<I2C_master>::Bus_status_flag a_f1,
                                                           Interrupt<I2C_master>::Bus_status_flag a_f2)
{
    return static_cast<Interrupt<I2C_master>::Bus_status_flag>(static_cast<std::uint32_t>(a_f1) |
                                                               static_cast<std::uint32_t>(a_f2));
}
constexpr Interrupt<I2C_master>::Bus_status_flag operator&(Interrupt<I2C_master>::Bus_status_flag a_f1,
                                                           Interrupt<I2C_master>::Bus_status_flag a_f2)
{
    return static_cast<Interrupt<I2C_master>::Bus_status_flag>(static_cast<std::uint32_t>(a_f1) &
                                                               static_cast<std::uint32_t>(a_f2));
}
constexpr Interrupt<I2C_master>::Bus_status_flag operator|=(Interrupt<I2C_master>::Bus_status_flag& a_f1,
                                                            Interrupt<I2C_master>::Bus_status_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr Interrupt<I2C_slave>::Bus_status_flag operator|(Interrupt<I2C_slave>::Bus_status_flag a_f1,
                                                          Interrupt<I2C_slave>::Bus_status_flag a_f2)
{
    return static_cast<Interrupt<I2C_slave>::Bus_status_flag>(static_cast<std::uint32_t>(a_f1) |
                                                              static_cast<std::uint32_t>(a_f2));
}
constexpr Interrupt<I2C_slave>::Bus_status_flag operator&(Interrupt<I2C_slave>::Bus_status_flag a_f1,
                                                          Interrupt<I2C_slave>::Bus_status_flag a_f2)
{
    return static_cast<Interrupt<I2C_slave>::Bus_status_flag>(static_cast<std::uint32_t>(a_f1) &
                                                              static_cast<std::uint32_t>(a_f2));
}
constexpr Interrupt<I2C_slave>::Bus_status_flag operator|=(Interrupt<I2C_slave>::Bus_status_flag& a_f1,
                                                           Interrupt<I2C_slave>::Bus_status_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc