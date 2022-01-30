#pragma once

/*
 *   Name: delay.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/Non_constructible.hpp>
#include <cml/utils/ms_tick_counter.hpp>
#include <cml/various.hpp>

#ifdef STM32L4
#include <cml/hal/cycles_counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/rcc.hpp>
#endif

namespace cml {
namespace utils {
class delay : private cml::Non_constructible
{
public:
    static void s(uint32_t a_time)
    {
        uint32_t start = ms_tick_counter::get();
        while (various::tick_diff(ms_tick_counter::get(), start) <= a_time * 1000u)
            ;
    }

    static void ms(uint32_t a_time)
    {
        uint32_t start = ms_tick_counter::get();
        while (various::tick_diff(ms_tick_counter::get(), start) <= a_time)
            ;
    }

#ifdef STM32L4
    inline static void us(uint32_t a_time)
    {
        hal::cycles_counter::reset();
        const std::uint32_t max =
            hal::cycles_counter::get() + (hal::rcc<hal::mcu>::get_SYSCLK_frequency_Hz() / 1_MHz * (a_time - 1));

        while (hal::cycles_counter::get() < max)
            ;
    }
#endif
};
} // namespace utils
} // namespace cml