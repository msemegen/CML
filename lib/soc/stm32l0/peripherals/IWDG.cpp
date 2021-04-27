/*
    Name: IWDG.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/peripherals/IWDG.hpp>

// soc
#ifdef CML_ASSERT_ENABLED
#include <soc/stm32l011xx/mcu.hpp>
#endif
#include <soc/system_timer.hpp>

// cml
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
namespace stm32l011xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

IWDG::IWDG()
{
    cml_assert(false == created);
    created = true;
}

bool IWDG::enable(Prescaler a_prescaler, uint16_t a_reload, const Window& a_window, uint32_t a_timeout)
{
    cml_assert((true == a_window.enable && a_window.value <= 0xFFFu) || (false == a_window.enable));
    cml_assert(true == mcu::is_clock_enabled(mcu::Clock::lsi));
    cml_assert(a_reload <= 0xFFFu);
    cml_assert(a_timeout > 0);

    uint32_t start = system_timer::get();

#ifndef IWDG
#define IWDG ((IWDG_TypeDef*)IWDG_BASE)
#endif // IWDG

    IWDG->KR = control_flags::enable;
    IWDG->KR = control_flags::write_access_enable;

    IWDG->PR = static_cast<uint32_t>(a_prescaler);
    bool ret = wait_until::all_bits(&(IWDG->SR), IWDG_SR_PVU, true, start, a_timeout);

    if (true == ret)
    {
        IWDG->RLR = a_reload;
        ret       = wait_until::all_bits(&(IWDG->SR), IWDG_SR_RVU, true, start, a_timeout);
    }

    if (true == ret)
    {
        if (true == a_window.enable)
        {
            IWDG->WINR = a_window.value;
            ret        = wait_until::all_bits(&(IWDG->SR), IWDG_SR_WVU, true, start, a_timeout);
        }
        else
        {
            IWDG->KR = control_flags::reload;
        }
    }

    if (false == ret)
    {
        this->disable();
    }

    return ret;

#ifdef IWDG
#undef IWDG
#endif // IWDG
}

void IWDG::disable()
{
#ifndef IWDG
#define IWDG ((IWDG_TypeDef*)IWDG_BASE)
#endif // IWDG

    IWDG->KR = 0;

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
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx