/*
    Name: watchdog.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <hal/stm32l452xx/independent_watchdog.hpp>

//cml
#include <debug/assert.hpp>
#include <hal/stm32l452xx/mcu.hpp>
#include <utils/sleep.hpp>

namespace {

using namespace cml::common;

struct control_flags
{
    static constexpr uint32 reload               = 0xAAAAu;
    static constexpr uint32 enable               = 0xCCCCu;
    static constexpr uint32 write_access_enable  = 0x5555u;
    static constexpr uint32 write_access_disable = 0x0000u;
};

} // namespace ::

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;
using namespace cml::utils;

bool independent_watchdog::enable(Prescaler a_prescaler,
                                  uint16 a_reload,
                                  const Window& a_window,
                                  time_tick a_timeout)
{
    assert((true == a_window.enable && a_window.value <= 0xFFFu) || (false == a_window.enable));
    assert(true == mcu::is_clock_enabled(mcu::Clock::lsi));
    assert(true == systick::is_enabled());
    assert(a_reload <= 0xFFFu);

    time_tick start = systick::get_counter();

    IWDG->KR = control_flags::enable;
    IWDG->KR = control_flags::write_access_enable;
    IWDG->PR = static_cast<uint32>(a_prescaler);
    IWDG->RLR = a_reload;

    bool ret = sleep::until(&(IWDG->SR), 0, false, start, a_timeout);

    if (true == ret)
    {
        if (true == a_window.enable)
        {
            IWDG->WINR = a_window.value;
        }
        else
        {
            IWDG->KR = control_flags::reload;
        }
    }
    else
    {
        disable();
    }

    return ret;
}

void independent_watchdog::disable()
{
    IWDG->KR = 0;
}

void independent_watchdog::feed()
{
    IWDG->KR = control_flags::reload;
}

} // namespace stm32l452xx
} // namespace cml
} // namespace hal