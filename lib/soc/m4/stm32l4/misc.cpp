/*
 *   Name: misc.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/misc.hpp>

// soc
#include <soc/m4/stm32l4/mcu/mcu.hpp>

// cml
#include <cml/various.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void misc::delay_us(std::uint32_t a_time)
{
    cml_assert(true == mcu::is_DWT_active());
    cml_assert(rcc<mcu>::get_SYSCLK_frequency_Hz() >= 1_MHz);
    cml_assert(a_time > 0);

    DWT->CYCCNT             = 0;
    const std::uint32_t max = DWT->CYCCNT + (rcc<mcu>::get_SYSCLK_frequency_Hz() / 1_MHz * (a_time - 1));
    while (DWT->CYCCNT < max)
        ;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4