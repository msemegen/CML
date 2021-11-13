#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/Basic_timer/Basic_timer.hpp>
#include <soc/m4/stm32l4/Basic_timer/Interrupt.hpp>
#include <soc/m4/stm32l4/Basic_timer/Polling.hpp>
#include <soc/m4/stm32l4/Factory.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

template<> class Factory<Basic_timer, 6u> : private cml::Non_constructible
{
public:
    static Basic_timer create()
    {
        return Basic_timer(0, TIM6);
    }
};
template<> class Factory<Interrupt<Basic_timer>, 6u> : private cml::Non_constructible
{
public:
    static Interrupt<Basic_timer> create(Basic_timer* a_p_timer)
    {
        return Interrupt<Basic_timer>(a_p_timer, IRQn_Type::TIM6_DAC_IRQn);
    }
};
template<> class Factory<Polling<Basic_timer>, 6u> : private cml::Non_constructible
{
public:
    static Polling<Basic_timer> create(Basic_timer* a_p_timer)
    {
        return Polling<Basic_timer>(a_p_timer);
    }
};
template<> void rcc<Basic_timer, 6>::enable(bool a_enable_in_lp);
template<> void rcc<Basic_timer, 6>::disable();
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
template<> class Factory<Basic_timer, 7u> : private cml::Non_constructible
{
public:
    static Basic_timer create()
    {
        return Basic_timer(1, TIM7);
    }
};
template<> class Factory<Interrupt<Basic_timer>, 7u> : private cml::Non_constructible
{
public:
    static Interrupt<Basic_timer> create(Basic_timer* a_p_timer)
    {
        return Interrupt<Basic_timer>(a_p_timer, IRQn_Type::TIM7_IRQn);
    }
};
template<> class Factory<Polling<Basic_timer>, 7u> : private cml::Non_constructible
{
public:
    static Polling<Basic_timer> create(Basic_timer* a_p_timer)
    {
        return Polling<Basic_timer>(a_p_timer);
    }
};
template<> void rcc<Basic_timer, 7>::enable(bool a_enable_in_lp);
template<> void rcc<Basic_timer, 7>::disable();
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc