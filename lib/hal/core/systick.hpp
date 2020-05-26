#pragma once

/*
    Name: systick.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>
#include <common/time.hpp>
#include <debug/assert.hpp>

namespace cml {
namespace hal {
namespace core {

class systick
{
public:

    static void enable(common::uint32 a_priority);
    static void disable();
    static void reset_counter();

    static bool is_enabled();
    static common::time::tick get_counter();

private:

    systick()               = delete;
    systick(systick&&)      = delete;
    systick(const systick&) = delete;
    ~systick()              = default;

    systick& operator = (systick&&)      = delete;
    systick& operator = (const systick&) = delete;
};

} // namespace core
} // namespace hal
} // namespace cml