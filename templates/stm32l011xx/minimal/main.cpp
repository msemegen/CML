/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/hal/systick.hpp>
#include <cml/utils/delay.hpp>

namespace {

using namespace cml::hal;

void system_timer_update(void*)
{
    system_timer::update();
}

} // namespace

int main()
{
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

    systick::enable((mcu::get_sysclk_frequency_hz() / 1000u) - 1, systick::Prescaler::_1, 0x9u);
    systick::register_tick_callback({ system_timer_update, nullptr });

    GPIO gpio_port_b(GPIO::Id::b);
    gpio_port_b.enable();

    GPIO::Out::Pin led_pin;
    gpio_port_b.p_out->enable(3u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);
    led_pin.set_level(GPIO::Level::low);

    while (true)
    {
        led_pin.toggle_level();
        delay::ms(500u);
    }
}