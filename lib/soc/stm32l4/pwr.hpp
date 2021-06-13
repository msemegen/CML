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
#include <cml/debug/assertion.hpp>

namespace soc {
namespace stm32l4 {

class pwr
{
public:
    enum Core_sleep_mode
    {
        sleep_on_exit,
        wfi,
        wfe,
    };

    enum class Deep_sleep
    {
        enable,
        disable
    };

    static void enable_core_sleep(Core_sleep_mode a_mode, Deep_sleep a_deep_sleep)
    {
        switch (a_mode)
        {
            case Core_sleep_mode::sleep_on_exit: {
                cml::bit_flag::set(&(SCB->SCR), SCB_SCR_SLEEPONEXIT_Msk);
            }
            break;

            case Core_sleep_mode::wfi: {
                __WFI();
            }
            break;

            case Core_sleep_mode::wfe: {
                __WFE();
            }
            break;
        }

        switch (a_deep_sleep)
        {
            case Deep_sleep::enable: {
                cml::bit_flag::set(&(SCB->SCR), SCB_SCR_SLEEPDEEP_Msk);
            }
            break;

            case Deep_sleep::disable: {
                cml::bit_flag::clear(&(SCB->SCR), SCB_SCR_SLEEPDEEP_Msk);
            }
            break;
        }
    }

    static void disable_core_sleep()
    {
        cml::bit_flag::clear(&(SCB->SCR), SCB_SCR_SLEEPONEXIT_Msk);
        cml::bit_flag::clear(&(SCB->SCR), SCB_SCR_SLEEPDEEP_Msk);
    }

    void peripheral_enable();

private:
    pwr()           = delete;
    pwr(const pwr&) = delete;
    pwr(pwr&&)      = delete;
    ~pwr()          = delete;

    pwr& operator=(const pwr&) = delete;
    pwr& operator=(pwr&&) = delete;

private:
    static Core_sleep_mode core_sleep_mode;
};

} // namespace stm32l4
} // namespace soc