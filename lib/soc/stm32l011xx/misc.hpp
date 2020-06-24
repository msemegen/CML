#pragma once

/*
    Name: misc.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/time.hpp>

namespace soc {
namespace stm32l011xx {

class misc
{
public:

    misc()            = delete;
    misc(misc&&)      = delete;
    misc(const misc&) = delete;

    misc& operator = (misc&&)      = delete;
    misc& operator = (const misc&) = delete;

    static void delay_us(cml::time::tick a_time);
};

} // namespace stm32l011xx
} // namespace soc