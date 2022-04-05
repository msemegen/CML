/*
 *   Name: bsp.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/GPIO/bsp.hpp>

// cml
#include <cml/bit_flag.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

#if defined(SOC_GPIOA_PRESENT)
template<> void rcc<GPIO, 1>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOASMEN);
    }
}
template<> void rcc<GPIO, 1>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOASMEN);
}
#endif

#if defined(SOC_GPIOB_PRESENT)
template<> void rcc<GPIO, 2>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOBSMEN);
    }
}
template<> void rcc<GPIO, 2>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOBSMEN);
}
#endif

#if defined(SOC_GPIOC_PRESENT)
template<> void rcc<GPIO, 3>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOCSMEN);
    }
}
template<> void rcc<GPIO, 3>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOCSMEN);
}
#endif

#if defined(SOC_GPIOD_PRESENT)
template<> void rcc<GPIO, 4>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIODSMEN);
    }
}
template<> void rcc<GPIO, 4>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIODSMEN);
}
#endif

#if defined(SOC_GPIOE_PRESENT)
template<> void rcc<GPIO, 5>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOESMEN);
    }
}
template<> void rcc<GPIO, 5>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOESMEN);
}
#endif

#if defined(SOC_GPIOH_PRESENT)
template<> void rcc<GPIO, 8>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOHSMEN);
    }
}
template<> void rcc<GPIO, 8>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOHSMEN);
}
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc