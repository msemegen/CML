/*
 *   Name: IWDG.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/peripherals/IWDG.hpp>

// soc
#include <soc/m4/stm32l4/mcu.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>

namespace {

using namespace cml;

struct control_flags
{
    static constexpr uint32_t reload               = 0xAAAAu;
    static constexpr uint32_t enable               = 0xCCCCu;
    static constexpr uint32_t write_access_enable  = 0x5555u;
    static constexpr uint32_t write_access_disable = 0x0000u;
};

bool created = false;

} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

IWDG::IWDG()
{
    cml_assert(false == created);
    created = true;
}

IWDG::~IWDG()
{
    created = false;
}

bool IWDG::enable(Prescaler a_prescaler, uint16_t a_reload, const Window& a_window, uint32_t a_timeout)
{
    cml_assert((true == a_window.enable && a_window.value <= 0xFFFu) || (false == a_window.enable));
    cml_assert(true == rcc<mcu>::is_clock_enabled(rcc<mcu>::Clock::LSI));
    cml_assert(a_reload <= 0xFFFu);
    cml_assert(a_timeout > 0);

#ifndef IWDG
#define IWDG ((IWDG_TypeDef*)IWDG_BASE)
#endif // IWDG

    uint32_t start = system_timer::get();

    IWDG->KR = control_flags::enable;
    IWDG->KR = control_flags::write_access_enable;

    IWDG->PR = static_cast<uint32_t>(a_prescaler);
    bool ret = wait_until::all_bits(&(IWDG->SR), IWDG_SR_PVU, true, start, a_timeout);

    if (true == ret)
    {
        IWDG->RLR = a_reload;
        ret = wait_until::all_bits(&(IWDG->SR), IWDG_SR_RVU, true, start, a_timeout - (system_timer::get() - start));
    }

    if (true == ret)
    {
        if (true == a_window.enable)
        {
            IWDG->WINR = a_window.value;
            ret =
                wait_until::all_bits(&(IWDG->SR), IWDG_SR_WVU, true, start, a_timeout - (system_timer::get() - start));
        }
        else
        {
            IWDG->KR = control_flags::reload;
        }
    }

    return ret;

#ifdef IWDG
#undef IWDG
#endif // IWDG
}

void IWDG::feed()
{
#ifndef IWDG
#define IWDG ((IWDG_TypeDef*)IWDG_BASE)
#endif // IWDG

    IWDG->KR = control_flags::reload;

#ifdef IWDG
#undef IWDG
#endif // IWDG
}

} // namespace peripherals
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4