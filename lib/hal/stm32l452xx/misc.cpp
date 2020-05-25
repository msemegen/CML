/*
    Name: misc.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <hal/stm32l452xx/misc.hpp>

//cml
#include <common/frequency.hpp>
#include <hal/stm32l452xx/mcu.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;

void misc::delay_us(time::tick a_time)
{
    assert(true == mcu::is_dwt_enabled());
    assert(mcu::get_sysclk_frequency_hz() >= MHz(1));
    assert(a_time > 0);

    DWT->CYCCNT = 0;
    const uint32 max = DWT->CYCCNT + (mcu::get_sysclk_frequency_hz() / MHz(1) * (a_time - 1));
    while (DWT->CYCCNT < max);
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml