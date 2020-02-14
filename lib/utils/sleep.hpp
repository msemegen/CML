#pragma once

/*
    Name: sleep.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/frequency.hpp>
#include <common/macros.hpp>
#include <common/time_tick.hpp>
#include <hal/MCU.hpp>
#include <hal/Systick.hpp>


namespace cml {
namespace utils {

struct sleep
{
    static void ms(common::time_tick a_time)
    {
        assert(true == hal::Systick::get_instance().is_enabled());

        common::time_tick start = hal::Systick::get_instance().get_counter();
        while (common::time_tick_diff(hal::Systick::get_instance().get_counter(), start) <= a_time);
    }

    inline static void us(common::time_tick a_time)
    {
        assert(true == hal::MCU::get_instance().is_dwt_enabled());
        assert(a_time > 0);

#ifdef CML_DWT_PRESENT
        DWT->CYCCNT = 0;
        const common::uint32 max = DWT->CYCCNT + (SystemCoreClock / common::MHz(1) * (a_time - 1));

        while (DWT->CYCCNT < max);
#endif
    }

    sleep()             = delete;
    sleep(sleep&&)      = delete;
    sleep(const sleep&) = delete;
    ~sleep()            = delete;

    sleep& operator = (sleep&&)      = delete;
    sleep& operator = (const sleep&) = delete;

private:

    static common::uint32 x;
};

common::uint32 sleep::x = 0;

} // namespace utils
} // namespace cml