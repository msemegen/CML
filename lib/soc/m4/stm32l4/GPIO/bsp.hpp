#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>
#include <soc/m4/stm32l4/defs.hpp>

namespace soc {
#if defined(SOC_GPIOA_PRESENT)
template<> class Peripheral<m4::stm32l4::GPIO, 1u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(0u, GPIOA);
    }
};
#endif

#if defined(SOC_GPIOB_PRESENT)
template<> class Peripheral<m4::stm32l4::GPIO, 2u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(1u, GPIOB);
    }
};
#endif

#if defined(SOC_GPIOC_PRESENT)
template<> class Peripheral<m4::stm32l4::GPIO, 3u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(2u, GPIOC);
    }
};
#endif

#if defined(SOC_GPIOD_PRESENT)
template<> class Peripheral<m4::stm32l4::GPIO, 4u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(3u, GPIOD);
    }
};
#endif

#if defined(SOC_GPIOE_PRESENT)
template<> class Peripheral<m4::stm32l4::GPIO, 5u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(4u, GPIOE);
    }
};
#endif

#if defined(SOC_GPIOH_PRESENT)
template<> class Peripheral<m4::stm32l4::GPIO, 8u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::GPIO create()
    {
        return m4::stm32l4::GPIO(7u, GPIOH);
    }
};
#endif
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {
#if defined(SOC_GPIOA_PRESENT)
template<> void rcc<GPIO, 1>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 1>::disable();
#endif

#if defined(SOC_GPIOB_PRESENT)
template<> void rcc<GPIO, 2>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 2>::disable();
#endif

#if defined(SOC_GPIOC_PRESENT)
template<> void rcc<GPIO, 3>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 3>::disable();
#endif

#if defined(SOC_GPIOD_PRESENT)
template<> void rcc<GPIO, 4>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 4>::disable();
#endif

#if defined(SOC_GPIOE_PRESENT)
template<> void rcc<GPIO, 5>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 5>::disable();
#endif

#if defined(SOC_GPIOH_PRESENT)
template<> void rcc<GPIO, 8>::enable(bool a_enable_in_lp);
template<> void rcc<GPIO, 8>::disable();
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc