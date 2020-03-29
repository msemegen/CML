#pragma once

/*
    Name: misc.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/time_tick.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

class misc
{
public:

    misc()            = delete;
    misc(misc&&)      = delete;
    misc(const misc&) = delete;

    misc& operator = (misc&&)      = delete;
    misc& operator = (const misc&) = delete;

    static void delay_us(common::time_tick a_time);
};

} // namespace stm32l011xx
} // namespace hal
} // namespace cml