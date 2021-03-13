/*
 *   Name: misc.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/misc.hpp>

// soc
#include <soc/stm32l011xx/mcu.hpp>

// cml
#include <cml/debug/assertion.hpp>

namespace soc {
namespace stm32l011xx {

using namespace cml;
using namespace soc::stm32l011xx;

void misc::delay_us(uint32_t a_time)
{
    cml_assert(mcu::get_sysclk_frequency_hz() >= 1u * 1000000u);
    cml_assert(a_time > 0);

    uint32_t count = ((((mcu::get_sysclk_frequency_hz() / 1u * 1000000u)) / 4) * (a_time - 1));

    __asm__ __volatile__("1: sub %0, #1 \n"
                         "   cmp %0, #0 \n"
                         "   bne  1b    \n"
                         : "+r"(count));
}

} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx