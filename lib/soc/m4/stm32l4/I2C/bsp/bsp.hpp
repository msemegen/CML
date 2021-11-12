#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/I2C/I2C.hpp>
#include <soc/m4/stm32l4/I2C/Interrupt.hpp>
#include <soc/m4/stm32l4/I2C/Polling.hpp>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/Factory.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

template<> class Factory<I2C_master, 1u> : private cml::Non_constructible
{
public:
    static I2C_master create()
    {
        return I2C_master(0, I2C1);
    }
};
template<> class Factory<Interrupt<I2C_master>, 1u> : private cml::Non_constructible
{
public:
    static Interrupt<I2C_master> create(I2C_master* a_p_SPI)
    {
        return Interrupt<I2C_master>(a_p_SPI, IRQn_Type::I2C1_EV_IRQn, IRQn_Type::I2C1_ER_IRQn);
    }
};
template<> class Factory<Polling<I2C_master>, 1u> : private cml::Non_constructible
{
public:
    static Polling<I2C_master> create(I2C_master* a_p_SPI)
    {
        return Polling<I2C_master>(a_p_SPI);
    }
};

template<> class Factory<I2C_slave, 1u> : private cml::Non_constructible
{
public:
    static I2C_slave create()
    {
        return I2C_slave(0, I2C1);
    }
};
template<> class Factory<Interrupt<I2C_slave>, 1u> : private cml::Non_constructible
{
public:
    static Interrupt<I2C_slave> create(I2C_slave* a_p_SPI)
    {
        return Interrupt<I2C_slave>(a_p_SPI, IRQn_Type::I2C1_EV_IRQn, IRQn_Type::I2C1_ER_IRQn);
    }
};
template<> class Factory<Polling<I2C_slave>, 1u> : private cml::Non_constructible
{
public:
    static Polling<I2C_slave> create(I2C_slave* a_p_SPI)
    {
        return Polling<I2C_slave>(a_p_SPI);
    }
};

template<> template<> void rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::HSI>(bool a_lp_enable);
template<> template<> void rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::PCLK1>(bool a_lp_enable);
template<> template<> void rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::SYSCLK>(bool a_lp_enable);
template<> void rcc<I2C, 1>::disable();

template<> class Factory<I2C_master, 2u> : private cml::Non_constructible
{
public:
    static I2C_master create()
    {
        return I2C_master(1, I2C2);
    }
};
template<> class Factory<Interrupt<I2C_master>, 2u> : private cml::Non_constructible
{
public:
    static Interrupt<I2C_master> create(I2C_master* a_p_SPI)
    {
        return Interrupt<I2C_master>(a_p_SPI, IRQn_Type::I2C2_EV_IRQn, IRQn_Type::I2C2_ER_IRQn);
    }
};
template<> class Factory<Polling<I2C_master>, 2u> : private cml::Non_constructible
{
public:
    static Polling<I2C_master> create(I2C_master* a_p_SPI)
    {
        return Polling<I2C_master>(a_p_SPI);
    }
};

template<> class Factory<I2C_slave, 2u> : private cml::Non_constructible
{
public:
    static I2C_slave create()
    {
        return I2C_slave(1, I2C1);
    }
};
template<> class Factory<Interrupt<I2C_slave>, 2u> : private cml::Non_constructible
{
public:
    static Interrupt<I2C_slave> create(I2C_slave* a_p_SPI)
    {
        return Interrupt<I2C_slave>(a_p_SPI, IRQn_Type::I2C2_EV_IRQn, IRQn_Type::I2C2_ER_IRQn);
    }
};
template<> class Factory<Polling<I2C_slave>, 2u> : private cml::Non_constructible
{
public:
    static Polling<I2C_slave> create(I2C_slave* a_p_SPI)
    {
        return Polling<I2C_slave>(a_p_SPI);
    }
};

template<> template<> void rcc<I2C, 2>::enable<rcc<I2C, 2>::Clock_source::HSI>(bool a_lp_enable);
template<> template<> void rcc<I2C, 2>::enable<rcc<I2C, 2>::Clock_source::PCLK1>(bool a_lp_enable);
template<> template<> void rcc<I2C, 2>::enable<rcc<I2C, 2>::Clock_source::SYSCLK>(bool a_lp_enable);
template<> void rcc<I2C, 2>::disable();

#if defined(STM32L412R8) || defined(STM32L431CB) || defined(STM32L412C8) || defined(STM32L412CB) || \
    defined(STM32L412RB) || defined(STM32L422CB) || defined(STM32L422RB) || defined(STM32L431CC) || \
    defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L433CB) || \
    defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || defined(STM32L433VC) || \
    defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || defined(STM32L451CC) || \
    defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || defined(STM32L451VC) || \
    defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || defined(STM32L452RC) || \
    defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || defined(STM32L462CE) || \
    defined(STM32L462RE) || defined(STM32L462VE)
template<> class Factory<I2C_master, 3u> : private cml::Non_constructible
{
public:
    static I2C_master create()
    {
        return I2C_master(2, I2C3);
    }
};
template<> class Factory<Interrupt<I2C_master>, 3u> : private cml::Non_constructible
{
public:
    static Interrupt<I2C_master> create(I2C_master* a_p_SPI)
    {
        return Interrupt<I2C_master>(a_p_SPI, IRQn_Type::I2C2_EV_IRQn, IRQn_Type::I2C2_ER_IRQn);
    }
};
template<> class Factory<Polling<I2C_master>, 3u> : private cml::Non_constructible
{
public:
    static Polling<I2C_master> create(I2C_master* a_p_SPI)
    {
        return Polling<I2C_master>(a_p_SPI);
    }
};

template<> class Factory<I2C_slave, 3u> : private cml::Non_constructible
{
public:
    static I2C_slave create()
    {
        return I2C_slave(2, I2C3);
    }
};
template<> class Factory<Interrupt<I2C_slave>, 3u> : private cml::Non_constructible
{
public:
    static Interrupt<I2C_slave> create(I2C_slave* a_p_SPI)
    {
        return Interrupt<I2C_slave>(a_p_SPI, IRQn_Type::I2C3_EV_IRQn, IRQn_Type::I2C3_ER_IRQn);
    }
};
template<> class Factory<Polling<I2C_slave>, 3u> : private cml::Non_constructible
{
public:
    static Polling<I2C_slave> create(I2C_slave* a_p_SPI)
    {
        return Polling<I2C_slave>(a_p_SPI);
    }
};

template<> template<> void rcc<I2C, 3>::enable<rcc<I2C, 3>::Clock_source::HSI>(bool a_lp_enable);
template<> template<> void rcc<I2C, 3>::enable<rcc<I2C, 3>::Clock_source::PCLK1>(bool a_lp_enable);
template<> template<> void rcc<I2C, 3>::enable<rcc<I2C, 3>::Clock_source::SYSCLK>(bool a_lp_enable);
template<> void rcc<I2C, 3>::disable();
#endif

#if defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || \
    defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || \
    defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || \
    defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
template<> class Factory<I2C_master, 4u> : private cml::Non_constructible
{
public:
    static I2C_master create()
    {
        return I2C_master(3, I2C4);
    }
};
template<> class Factory<Interrupt<I2C_master>, 4u> : private cml::Non_constructible
{
public:
    static Interrupt<I2C_master> create(I2C_master* a_p_SPI)
    {
        return Interrupt<I2C_master>(a_p_SPI, IRQn_Type::I2C4_EV_IRQn, IRQn_Type::I2C4_ER_IRQn);
    }
};
template<> class Factory<Polling<I2C_master>, 4u> : private cml::Non_constructible
{
public:
    static Polling<I2C_master> create(I2C_master* a_p_SPI)
    {
        return Polling<I2C_master>(a_p_SPI);
    }
};

template<> class Factory<I2C_slave, 4u> : private cml::Non_constructible
{
public:
    static I2C_slave create()
    {
        return I2C_slave(3, I2C4);
    }
};
template<> class Factory<Interrupt<I2C_slave>, 4u> : private cml::Non_constructible
{
public:
    static Interrupt<I2C_slave> create(I2C_slave* a_p_SPI)
    {
        return Interrupt<I2C_slave>(a_p_SPI, IRQn_Type::I2C4_EV_IRQn, IRQn_Type::I2C4_ER_IRQn);
    }
};
template<> class Factory<Polling<I2C_slave>, 4u> : private cml::Non_constructible
{
public:
    static Polling<I2C_slave> create(I2C_slave* a_p_SPI)
    {
        return Polling<I2C_slave>(a_p_SPI);
    }
};

template<> template<> void rcc<I2C, 4>::enable<rcc<I2C, 4>::Clock_source::HSI>(bool a_lp_enable);
template<> template<> void rcc<I2C, 4>::enable<rcc<I2C, 4>::Clock_source::PCLK1>(bool a_lp_enable);
template<> template<> void rcc<I2C, 4>::enable<rcc<I2C, 4>::Clock_source::SYSCLK>(bool a_lp_enable);
template<> void rcc<I2C, 4>::disable();
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc