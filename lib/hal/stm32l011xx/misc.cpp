/*
    Name: misc.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <hal/stm32l011xx/misc.hpp>

//cml
#include <common/frequency.hpp>
#include <hal/stm32l011xx/mcu.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

using namespace cml::common;

void misc::delay_us(time::tick a_time)
{
    assert(mcu::get_sysclk_frequency_hz() >= MHz(1));
    assert(a_time > 0);

    uint32 count = ((((mcu::get_sysclk_frequency_hz() / MHz(1))) / 4) * (a_time - 1));

    __asm__ __volatile__("1: sub %0, #1 \n"
                         "   cmp %0, #0 \n"
                         "   bne  1b    \n"
                         : "+r" (count));
}

} // namespace stm32l011xx
} // namespace hal
} // namespace cml