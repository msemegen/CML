#pragma once

/*
    Name: iwdg.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l452xx.h>

//cml
#include <common/integer.hpp>
#include <common/time.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class iwdg
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

    iwdg()            = delete;
    iwdg(iwdg&&)      = delete;
    iwdg(const iwdg&) = delete;

    iwdg& operator = (iwdg&&)      = delete;
    iwdg& operator = (const iwdg&) = delete;

    static bool enable(Prescaler a_prescaler,
                       common::uint16 a_reload,
                       const Window& a_window,
                       common::time::tick a_timeout);

    static void feed();
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml