#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/USART/Interrupt.hpp>
#include <soc/m4/stm32l4/USART/Polling.hpp>
#include <soc/m4/stm32l4/USART/RS485.hpp>
#include <soc/m4/stm32l4/USART/USART.hpp>

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
template<> class Factory<USART, 1u> : private cml::Non_constructible
{
public:
    static USART create()
    {
        return USART(0u, USART1);
    }
};
template<> class Factory<Interrupt<USART>, 1u> : private cml::Non_constructible
{
public:
    static Interrupt<USART> create(USART* a_p_USART)
    {
        return Interrupt<USART>(a_p_USART, IRQn_Type::USART1_IRQn);
    }
};
template<> class Factory<Polling<USART>, 1u> : private cml::Non_constructible
{
public:
    static Polling<USART> create(USART* a_p_USART)
    {
        return Polling<USART>(a_p_USART);
    }
};
template<> class Factory<RS485, 1u> : private cml::Non_constructible
{
public:
    static RS485 create()
    {
        return RS485(1u, USART1);
    }
};
template<> class Factory<Interrupt<RS485>, 1u> : private cml::Non_constructible
{
public:
    static Interrupt<RS485> create(RS485* a_p_RS485)
    {
        return Interrupt<RS485>(a_p_RS485, IRQn_Type::USART1_IRQn);
    }
};
template<> class Factory<Polling<RS485>, 1u> : private cml::Non_constructible
{
public:
    static Polling<RS485> create(RS485* a_p_URS485, GPIO::Out::Pin* a_p_flow_control_pin)
    {
        return Polling<RS485>(a_p_URS485, a_p_flow_control_pin);
    }
};
template<> template<> void rcc<USART, 1u>::enable<rcc<USART, 1u>::Clock_source::HSI>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 1u>::enable<rcc<USART, 1u>::Clock_source::PCLK1>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 1u>::enable<rcc<USART, 1u>::Clock_source::SYSCLK>(bool a_enable_in_lp);
template<> void rcc<USART, 1u>::disable();

template<> class Factory<USART, 2u> : private cml::Non_constructible
{
public:
    static USART create()
    {
        return USART(1u, USART2);
    }
};
template<> class Factory<Interrupt<USART>, 2u> : private cml::Non_constructible
{
public:
    static Interrupt<USART> create(USART* a_p_USART)
    {
        return Interrupt<USART>(a_p_USART, IRQn_Type::USART2_IRQn);
    }
};
template<> class Factory<Polling<USART>, 2u> : private cml::Non_constructible
{
public:
    static Polling<USART> create(USART* a_p_USART)
    {
        return Polling<USART>(a_p_USART);
    }
};
template<> class Factory<RS485, 2u> : private cml::Non_constructible
{
public:
    static RS485 create()
    {
        return RS485(1u, USART2);
    }
};
template<> class Factory<Interrupt<RS485>, 2u> : private cml::Non_constructible
{
public:
    static Interrupt<RS485> create(RS485* a_p_RS485)
    {
        return Interrupt<RS485>(a_p_RS485, IRQn_Type::USART1_IRQn);
    }
};
template<> class Factory<Polling<RS485>, 2u> : private cml::Non_constructible
{
public:
    static Polling<RS485> create(RS485* a_p_URS485, GPIO::Out::Pin* a_p_flow_control_pin)
    {
        return Polling<RS485>(a_p_URS485, a_p_flow_control_pin);
    }
};
template<> template<> void rcc<USART, 2u>::enable<rcc<USART, 2u>::Clock_source::HSI>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 2u>::enable<rcc<USART, 2u>::Clock_source::PCLK1>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 2u>::enable<rcc<USART, 2u>::Clock_source::SYSCLK>(bool a_enable_in_lp);
template<> void rcc<USART, 2u>::disable();

#if defined(STM32L412R8) || defined(STM32L431CB) || defined(STM32L412C8) || defined(STM32L412CB) || \
    defined(STM32L412RB) || defined(STM32L422CB) || defined(STM32L422RB) || defined(STM32L431CC) || \
    defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L433CB) || \
    defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || defined(STM32L433VC) || \
    defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || defined(STM32L451CC) || \
    defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || defined(STM32L451VC) || \
    defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || defined(STM32L452RC) || \
    defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || defined(STM32L462CE) || \
    defined(STM32L462RE) || defined(STM32L462VE)
template<> class Factory<USART, 3u> : private cml::Non_constructible
{
public:
    static USART create()
    {
        return USART(2u, USART3);
    }
};
template<> class Factory<Interrupt<USART>, 3u> : private cml::Non_constructible
{
public:
    static Interrupt<USART> create(USART* a_p_USART)
    {
        return Interrupt<USART>(a_p_USART, IRQn_Type::USART3_IRQn);
    }
};
template<> class Factory<Polling<USART>, 3u> : private cml::Non_constructible
{
public:
    static Polling<USART> create(USART* a_p_USART)
    {
        return Polling<USART>(a_p_USART);
    }
};
template<> class Factory<RS485, 3u> : private cml::Non_constructible
{
public:
    static RS485 create()
    {
        return RS485(2u, USART3);
    }
};
template<> class Factory<Interrupt<RS485>, 3u> : private cml::Non_constructible
{
public:
    static Interrupt<RS485> create(RS485* a_p_RS485)
    {
        return Interrupt<RS485>(a_p_RS485, IRQn_Type::USART1_IRQn);
    }
};
template<> class Factory<Polling<RS485>, 3u> : private cml::Non_constructible
{
public:
    static Polling<RS485> create(RS485* a_p_URS485, GPIO::Out::Pin* a_p_flow_control_pin)
    {
        return Polling<RS485>(a_p_URS485, a_p_flow_control_pin);
    }
};
template<> template<> void rcc<USART, 3u>::enable<rcc<USART, 3u>::Clock_source::HSI>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 3u>::enable<rcc<USART, 3u>::Clock_source::PCLK1>(bool a_enable_in_lp);
template<> template<> void rcc<USART, 3u>::enable<rcc<USART, 3u>::Clock_source::SYSCLK>(bool a_enable_in_lp);
template<> void rcc<USART, 3>::disable();
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc