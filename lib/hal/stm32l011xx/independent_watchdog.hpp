#pragma once

/*
    Name: watchdog.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l011xx.h>

//cml
#include <common/integer.hpp>
#include <common/time_tick.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

class independent_watchdog
{
public:

    enum class Prescaler : common::uint32
    {
        _4   = 0x0u,
        _8   = IWDG_PR_PR_0,
        _16  = IWDG_PR_PR_1,
        _32  = IWDG_PR_PR_1 | IWDG_PR_PR_0,
        _64  = IWDG_PR_PR_2,
        _128 = IWDG_PR_PR_2 | IWDG_PR_PR_0,
        _256 = IWDG_PR_PR_2 | IWDG_PR_PR_1
    };

    struct Window
    {
        bool enable          = false;
        common::uint16 value = 0xFFFu;
    };

public:

    independent_watchdog()                            = delete;
    independent_watchdog(independent_watchdog&&)      = delete;
    independent_watchdog(const independent_watchdog&) = delete;

    independent_watchdog& operator = (independent_watchdog&&)      = delete;
    independent_watchdog& operator = (const independent_watchdog&) = delete;

    static bool enable(Prescaler a_prescaler,
                       common::uint16 a_reload,
                       const Window& a_window,
                       common::time_tick a_timeout);

    static void disable();
    static void feed();
};

} // namespace stm32l011xx
} // namespace hal
} // namespace cml