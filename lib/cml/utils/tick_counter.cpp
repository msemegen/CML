/*
 *   Name: tick_counter.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <cml/hal/mcu.hpp>
#include <cml/hal/rcc.hpp>
#include <cml/utils/tick_counter.hpp>

namespace {
cml::Milliseconds cnt;
void* p_timer = nullptr;
} // namespace

namespace cml {
namespace utils {
using namespace hal;

Milliseconds tick_counter::get()
{
    return cnt;
}

void tick_counter::update(void*)
{
    ++cnt;
}

template<> void tick_counter::enable<Systick>(Systick* a_p_timer, const IRQ_config& a_irq_config)
{
    a_p_timer->enable((rcc<mcu>::get_HCLK_frequency_Hz() / 1000u) - 1, Systick::Prescaler::_1);
    a_p_timer->interrupt.enable({ 0x1u, 0x1u });
    a_p_timer->interrupt.register_callback({ tick_counter::update, nullptr });
    p_timer = a_p_timer;
}

template<> void tick_counter::disable<Systick>()
{
    cml_assert(nullptr != p_timer);

    (reinterpret_cast<Systick*>(p_timer))->disable();
    p_timer = nullptr;
    cnt     = 0_ms;
}
} // namespace utils
} // namespace cml