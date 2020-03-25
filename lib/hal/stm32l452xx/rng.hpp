#pragma once

/*
    Name: rng.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//external
#include <stm32l452xx.h>

//cml
#include <common/bit.hpp>
#include <common/time_tick.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class rng
{
public:

    using New_value_callback = void(*)(common::uint32 a_value, bool a_clock_error, bool a_seed_error);

    rng()           = delete;
    rng(rng&&)      = delete;
    rng(const rng&) = delete;

    rng& operator = (rng&&)      = delete;
    rng& operator = (const rng&) = delete;

    static bool enable(common::uint32 a_irq_priority, common::time_tick a_timeout_ms);
    static void disable();

    static bool get_value_polling(common::uint32* a_p_value, common::time_tick a_timeout_ms);

    static void get_value_it(New_value_callback a_callback);
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml