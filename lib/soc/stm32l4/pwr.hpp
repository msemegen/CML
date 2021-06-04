#pragma once

/*
 *   Name: pwr.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// cml
#include <cml/bit_flag.hpp>

namespace soc {
namespace stm32l4 {

class pwr
{
public:
    static void enable_sleep_on_exit()
    {
        cml::bit_flag::set(&(SCB->SCR), SCB_SCR_SLEEPONEXIT_Msk);
    }

    static void disable_sleep_on_exit()
    {
        cml::bit_flag::clear(&(SCB->SCR), SCB_SCR_SLEEPONEXIT_Msk);
    }

private:
    pwr()           = delete;
    pwr(const pwr&) = delete;
    pwr(pwr&&)      = delete;
    ~pwr()          = delete;

    pwr& operator=(const pwr&) = delete;
    pwr& operator=(pwr&&) = delete;
};

} // namespace stm32l4
} // namespace soc