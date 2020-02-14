#pragma once

/*
    Name: systick.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>
#include <common/time_tick.hpp>

namespace cml {
namespace hal {

class systick
{
public:

    static void enable(common::uint32 a_priority);
    static void disable();
    static void reset_counter();

    static bool is_enabled();
    static common::time_tick get_counter();

private:

    systick()               = delete;
    systick(systick&&)      = delete;
    systick(const systick&) = delete;
    ~systick()              = default;

    systick& operator = (systick&&)      = delete;
    systick& operator = (const systick&) = delete;

};

} // namespace hal
} // namespace cml