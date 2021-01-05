/*
    Name: iwdg.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <soc/stm32l452xx/system/iwdg.hpp>

// soc
#include <soc/stm32l452xx/mcu.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assert.hpp>
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

} // namespace

namespace soc {
namespace stm32l452xx {
namespace system {

using namespace cml;
using namespace cml::utils;

bool iwdg::enable(Prescaler a_prescaler, uint16_t a_reload, const Window& a_window, time::tick a_timeout)
{
    assert((true == a_window.enable && a_window.value <= 0xFFFu) || (false == a_window.enable));
    assert(true == mcu::is_clock_enabled(mcu::Clock::lsi));
    assert(a_reload <= 0xFFFu);
    assert(a_timeout > 0);

    time::tick start = system_timer::get();

    IWDG->KR = control_flags::enable;
    IWDG->KR = control_flags::write_access_enable;

    IWDG->PR = static_cast<uint32_t>(a_prescaler);
    bool ret = wait_until::flag(&(IWDG->SR), IWDG_SR_PVU, true, start, a_timeout);

    if (true == ret)
    {
        IWDG->RLR = a_reload;
        ret       = wait_until::flag(&(IWDG->SR), IWDG_SR_RVU, true, start, a_timeout);
    }

    if (true == ret)
    {
        if (true == a_window.enable)
        {
            IWDG->WINR = a_window.value;
            ret        = wait_until::flag(&(IWDG->SR), IWDG_SR_WVU, true, start, a_timeout);
        }
        else
        {
            IWDG->KR = control_flags::reload;
        }
    }

    return ret;
}

void iwdg::feed()
{
    IWDG->KR = control_flags::reload;
}

} // namespace system
} // namespace stm32l452xx
} // namespace soc