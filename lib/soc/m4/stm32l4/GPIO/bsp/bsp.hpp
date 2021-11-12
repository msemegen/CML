#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/Factory.hpp>
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>
#include <soc/m4/stm32l4/GPIO/Interrupt.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Factory<GPIO, 1u> : private cml::Non_constructible
{
public:
    static GPIO create()
    {
        return GPIO(0u, GPIOA);
    }
};
template<> void rcc<GPIO, 1>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 1>::disable();

template<> class Factory<GPIO, 2u> : private cml::Non_constructible
{
public:
    static GPIO create()
    {
        return GPIO(1u, GPIOB);
    }
};
template<> void rcc<GPIO, 2>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 2>::disable();

template<> class Factory<GPIO, 3u> : private cml::Non_constructible
{
public:
    static GPIO create()
    {
        return GPIO(2u, GPIOC);
    }
};
template<> void rcc<GPIO, 3>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 3>::disable();

template<> class Factory<GPIO, 8u> : private cml::Non_constructible
{
public:
    static GPIO create()
    {
        return GPIO(7u, GPIOH);
    }
};
template<> void rcc<GPIO, 8>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 8>::disable();

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> class Factory<GPIO, 4u> : private cml::Non_constructible
{
public:
    static GPIO create()
    {
        return GPIO(3u, GPIOD);
    }
};
template<> void rcc<GPIO, 4>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 4>::disable();
#endif
#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
template<> class Factory<GPIO, 5u> : private cml::Non_constructible
{
public:
    static GPIO create()
    {
        return GPIO(4u, GPIOE);
    }
};
template<> void rcc<GPIO, 5>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 5>::disable();
#endif

template<> class Factory<Interrupt<GPIO>> : private cml::Non_constructible
{
public:
    template<Interrupt<GPIO>::Id> static Interrupt<GPIO> create();
};

template<> Interrupt<GPIO> Factory<Interrupt<GPIO>>::create<Interrupt<GPIO>::Id::_0>();
template<> Interrupt<GPIO> Factory<Interrupt<GPIO>>::create<Interrupt<GPIO>::Id::_1>();
template<> Interrupt<GPIO> Factory<Interrupt<GPIO>>::create<Interrupt<GPIO>::Id::_2>();
template<> Interrupt<GPIO> Factory<Interrupt<GPIO>>::create<Interrupt<GPIO>::Id::_3>();
template<> Interrupt<GPIO> Factory<Interrupt<GPIO>>::create<Interrupt<GPIO>::Id::_4>();
template<> Interrupt<GPIO> Factory<Interrupt<GPIO>>::create<Interrupt<GPIO>::Id::_5_9>();
template<> Interrupt<GPIO> Factory<Interrupt<GPIO>>::create<Interrupt<GPIO>::Id::_10_15>();
} // namespace stm32l4
} // namespace m4
} // namespace soc