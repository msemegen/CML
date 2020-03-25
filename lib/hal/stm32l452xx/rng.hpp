#pragma once

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

    rng()           = delete;
    rng(rng&&)      = delete;
    rng(const rng&) = delete;

    rng& operator = (rng&&)      = delete;
    rng& operator = (const rng&) = delete;

    static bool enable(common::uint32 a_irq_priority, common::time_tick a_timeout_ms);
    static void disable();

    static bool get_value_polling(common::uint32* a_p_value, common::time_tick a_timeout_ms);
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml