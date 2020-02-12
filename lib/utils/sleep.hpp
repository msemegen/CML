#pragma once

/*
    Name: sleep.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/time_tick.hpp>
#include <hal/Systick.hpp>

namespace cml {
namespace utils {

void sleep(common::time_tick a_time_ms)
{
    assert(true == hal::Systick::get_instance().is_enabled());

    common::time_tick start = hal::Systick::get_instance().get_counter();
    while (common::time_tick_diff(hal::Systick::get_instance().get_counter(), start) <= a_time_ms);
}

} // namespace utils
} // namespace cml