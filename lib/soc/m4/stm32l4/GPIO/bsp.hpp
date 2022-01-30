#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>

namespace soc {
template<> class Peripheral<m4::stm32l4::GPIO, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(0u, GPIOA);
    }
};

template<> class Peripheral<m4::stm32l4::GPIO, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(1u, GPIOB);
    }
};

template<> class Peripheral<m4::stm32l4::GPIO, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(2u, GPIOC);
    }
};

template<> class Peripheral<m4::stm32l4::GPIO, 8u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(7u, GPIOH);
    }
};

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> class Peripheral<m4::stm32l4::GPIO, 4u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(3u, GPIOD);
    }
};
#endif

#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
template<> class Peripheral<m4::stm32l4::GPIO, 5u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(4u, GPIOE);
    }
};
#endif
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> void rcc<GPIO, 1>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 1>::disable();

template<> void rcc<GPIO, 2>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 2>::disable();

template<> void rcc<GPIO, 3>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 3>::disable();

template<> void rcc<GPIO, 8>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 8>::disable();

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<GPIO, 4>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 4>::disable();
#endif
#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<GPIO, 5>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 5>::disable();
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc