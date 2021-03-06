/*
 *   Name: misc.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/misc.hpp>

// soc
#include <soc/stm32l452xx/mcu.hpp>

// cml
#include <cml/debug/assertion.hpp>

namespace soc {
namespace stm32l452xx {

using namespace cml;

void misc::delay_us(uint32_t a_time)
{
    cml_assert(true == mcu::is_dwt_enabled());
    cml_assert(mcu::get_sysclk_frequency_hz() >= 1000000u);
    cml_assert(a_time > 0);

    DWT->CYCCNT        = 0;
    const uint32_t max = DWT->CYCCNT + (mcu::get_sysclk_frequency_hz() / 1000000u * (a_time - 1));
    while (DWT->CYCCNT < max)
        ;
}

} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx