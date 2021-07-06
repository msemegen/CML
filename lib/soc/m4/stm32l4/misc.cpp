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
#include <soc/m4/stm32l4/mcu.hpp>

// cml
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void misc::delay_us(uint32_t a_time)
{
    cml_assert(mcu::DWT_mode::enabled == mcu::get_dwt_mode());
    cml_assert(rcc<mcu>::get_sysclk_frequency_hz() >= 1000000u);
    cml_assert(a_time > 0);

    DWT->CYCCNT        = 0;
    const uint32_t max = DWT->CYCCNT + (rcc<mcu>::get_sysclk_frequency_hz() / 1000000u * (a_time - 1));
    while (DWT->CYCCNT < max)
        ;
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4