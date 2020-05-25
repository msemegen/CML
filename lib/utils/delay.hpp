#pragma once

/*
    Name: delay.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/time.hpp>
#include <debug/assert.hpp>
#include <hal/systick.hpp>

#ifdef STM32L452xx
#include <hal/stm32l452xx/misc.hpp>
#endif // STM32L452xx

#ifdef STM32L011xx
#include <hal/stm32l011xx/misc.hpp>
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

    delay& operator = (delay&&)      = delete;
    delay& operator = (const delay&) = delete;

    static void ms(common::time::tick a_time)
    {
        assert(true == hal::systick::is_enabled());

        common::time::tick start = hal::systick::get_counter();
        while (common::time::diff(hal::systick::get_counter(), start) <= a_time);
    }

    inline static void us(common::time::tick a_time)
    {
#ifdef STM32L452xx
        hal::stm32l452xx::misc::delay_us(a_time);
#endif // STM32L452xx

#ifdef STM32L011xx
        hal::stm32l011xx::misc::delay_us(a_time);
#endif // STM32L011xx
    }
};

} // namespace utils
} // namespace cml