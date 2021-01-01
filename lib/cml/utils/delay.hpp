#pragma once

/*
    Name: delay.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// cml
#include <cml/debug/assert.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/time.hpp>

#ifdef STM32L452xx
#include <soc/stm32l452xx/misc.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <soc/stm32l011xx/misc.hpp>
#endif // STM32L011xx

namespace cml {
namespace utils {

class delay
{
public:
    delay()             = delete;
    delay(delay&&)      = delete;
    delay(const delay&) = delete;
    ~delay()            = delete;

    delay& operator=(delay&&) = delete;
    delay& operator=(const delay&) = delete;

    static void ms(time::tick a_time)
    {
        time::tick start = hal::system_timer::get();
        while (time::diff(hal::system_timer::get(), start) <= a_time)
            ;
    }

    inline static void us(time::tick a_time)
    {
#ifdef STM32L452xx
        soc::stm32l452xx::misc::delay_us(a_time);
#endif // STM32L452xx

#ifdef STM32L011xx
        soc::stm32l011xx::misc::delay_us(a_time);
#endif // STM32L011xx
    }
};

} // namespace utils
} // namespace cml