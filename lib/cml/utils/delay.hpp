#pragma once

/*
 *   Name: delay.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/hal/system_timer.hpp>
#include <cml/various.hpp>

#ifdef STM32L4
#include <soc/m4/stm32l4/misc.hpp>
#endif // STM32L4

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

    static void s(uint32_t a_time)
    {
        uint32_t start = hal::system_timer::get();
        while (various::time_diff(hal::system_timer::get(), start) <= a_time * 1000u)
            ;
    }

    static void ms(uint32_t a_time)
    {
        uint32_t start = hal::system_timer::get();
        while (various::time_diff(hal::system_timer::get(), start) <= a_time)
            ;
    }

    inline static void us(uint32_t a_time)
    {
#ifdef STM32L4
        soc::m4::stm32l4::misc::delay_us(a_time);
#endif // STM32L4
    }
};

} // namespace utils
} // namespace cml