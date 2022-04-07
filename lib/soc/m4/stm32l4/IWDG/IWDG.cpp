/*
 *   Name: IWDG.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/IWDG/IWDG.hpp>

// soc
#include <soc/m4/stm32l4/mcu/mcu.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/tick_counter.hpp>
#include <cml/utils/wait_until.hpp>

namespace {
using namespace cml;

struct control_flags : private Non_constructible
{
    static constexpr std::uint32_t reload               = 0xAAAAu;
    static constexpr std::uint32_t enable               = 0xCCCCu;
    static constexpr std::uint32_t write_access_enable  = 0x5555u;
    static constexpr std::uint32_t write_access_disable = 0x0000u;
};
} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
#define IWDG_T ((IWDG_TypeDef*)IWDG_BASE)

using namespace cml;
using namespace cml::utils;

bool IWDG::enable(Prescaler a_prescaler, std::uint16_t a_reload, const Window& a_window, Milliseconds a_timeout)
{
    cml_assert(true == this->is_created());

    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);
    cml_assert(various::get_enum_incorrect_value<Window::Mode>() != a_window.mode);
    cml_assert((Window::Mode::enabled == a_window.mode && a_window.value <= 0xFFFu) ||
               (Window::Mode::disabled == a_window.mode));
    cml_assert(true == rcc<mcu>::LSI::is_enabled());
    cml_assert(a_reload <= 0xFFFu);
    cml_assert(a_timeout > 0_ms);

    Milliseconds start = tick_counter::get();

    IWDG_T->KR = control_flags::enable;
    IWDG_T->KR = control_flags::write_access_enable;

    IWDG_T->PR = static_cast<std::uint32_t>(a_prescaler);
    bool ret   = wait_until::all_bits(&(IWDG_T->SR), IWDG_SR_PVU, true, start, a_timeout);

    if (true == ret)
    {
        IWDG_T->RLR = a_reload;
        ret =
            wait_until::all_bits(&(IWDG_T->SR), IWDG_SR_RVU, true, start, a_timeout - (tick_counter::get() - start));
    }

    if (true == ret)
    {
        if (Window::Mode::enabled == a_window.mode)
        {
            IWDG_T->WINR = a_window.value;
            ret          = wait_until::all_bits(
                &(IWDG_T->SR), IWDG_SR_WVU, true, start, a_timeout - (tick_counter::get() - start));
        }
        else
        {
            IWDG_T->KR = control_flags::reload;
        }
    }

    return ret;
}

void IWDG::feed()
{
    cml_assert(true == this->is_created());

    IWDG_T->KR = control_flags::reload;
}
bool IWDG::is_enabled() const
{
    return 0x0u != IWDG_T->KR;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4