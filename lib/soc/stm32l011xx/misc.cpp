/*
    Name: misc.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/misc.hpp>

// soc
#include <soc/stm32l011xx/mcu.hpp>

// cml
#include <cml/debug/assert.hpp>
#include <cml/frequency.hpp>

namespace soc {
namespace stm32l011xx {

using namespace cml;
using namespace soc::stm32l011xx;

void misc::delay_us(time::tick a_time)
{
    assert(mcu::get_sysclk_frequency_hz() >= MHz_to_Hz(1));
    assert(a_time > 0);

    uint32_t count = ((((mcu::get_sysclk_frequency_hz() / MHz_to_Hz(1))) / 4) * (a_time - 1));

    __asm__ __volatile__("1: sub %0, #1 \n"
                         "   cmp %0, #0 \n"
                         "   bne  1b    \n"
                         : "+r"(count));
}

} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx