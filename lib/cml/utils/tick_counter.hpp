#pragma once

/*
 *   Name: tick_counter.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// cml
#include <cml/Duration.hpp>
#include <cml/Non_constructible.hpp>
#include <cml/hal/Basic_timer.hpp>
#include <cml/hal/IRQ_config.hpp>
#include <cml/hal/Systick.hpp>

#ifdef STM32L4
#include <cml/hal/cycles_counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/rcc.hpp>
#endif

namespace cml {
namespace utils {
class tick_counter : private cml::Non_constructible
{
public:
    template<typename Timer_t> static void enable(Timer_t* a_p_timer, const hal::IRQ_config& a_irq_config) = delete;
    template<typename Timer_t> static void disable()                                                       = delete;

    static Milliseconds get();

    static void delay(Milliseconds a_time)
    {
        Milliseconds start = tick_counter::get();
        while (tick_counter::get() - start <= a_time)
            ;
    }

    static void delay(Seconds a_time)
    {
        Milliseconds start = tick_counter::get();
        while (tick_counter::get() - start <= a_time)
            ;
    }

#ifdef STM32L4
    static void delay(Microseconds a_time)
    {
        cml_assert(hal::rcc<hal::mcu>::get_SYSCLK_frequency_Hz() >= 1_MHz);

        hal::cycles_counter::reset();
        const std::uint32_t max = hal::cycles_counter::get() +
                                  (hal::rcc<hal::mcu>::get_SYSCLK_frequency_Hz() / 1_MHz * (a_time - 1_us).get());

        while (hal::cycles_counter::get() < max)
            ;
    }
#endif

private:
    static void update(void*);
};

template<> void tick_counter::enable<hal::Systick>(hal::Systick* a_p_timer, const hal::IRQ_config& a_irq_config);
template<> void tick_counter::disable<hal::Systick>();

template<>
void tick_counter::enable<hal::Basic_timer>(hal::Basic_timer* a_p_timer, const hal::IRQ_config& a_irq_config);
template<> void tick_counter::disable<hal::Basic_timer>();
} // namespace utils
} // namespace cml
