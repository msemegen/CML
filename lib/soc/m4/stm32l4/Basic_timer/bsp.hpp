#pragma once

/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/Basic_timer/Basic_timer.hpp>

namespace soc {
template<> class Peripheral<m4::stm32l4::Basic_timer, 6u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Basic_timer create()
    {
        return m4::stm32l4::Basic_timer(0, TIM6, IRQn_Type::TIM6_DAC_IRQn);
    }
};

#if defined(STM32L431CC) || defined(STM32L431KB) || defined(STM32L431KC) || defined(STM32L431RB) || \
    defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L432KB) || defined(STM32L432KC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L442KC) || defined(STM32L443CC) || defined(STM32L443RC) || \
    defined(STM32L443VC)
template<> class Peripheral<m4::stm32l4::Basic_timer, 7u> : private cml::Non_constructible
{
public:
    static m4::stm32l4::Basic_timer create()
    {
        return m4::stm32l4::Basic_timer(1, TIM7, IRQn_Type::TIM7_IRQn);
    }
};
#endif
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> void rcc<Basic_timer, 6>::enable(bool a_enable_in_lp);
template<> void rcc<Basic_timer, 6>::disable();
#if defined(STM32L431CC) || defined(STM32L431KB) || defined(STM32L431KC) || defined(STM32L431RB) || \
    defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L432KB) || defined(STM32L432KC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L442KC) || defined(STM32L443CC) || defined(STM32L443RC) || \
    defined(STM32L443VC)
template<> void rcc<Basic_timer, 7>::enable(bool a_enable_in_lp);
template<> void rcc<Basic_timer, 7>::disable();
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc