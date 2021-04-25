/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/debug/assertion.hpp>
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

void assert_halt(void*)
{
    mcu::halt();
    while (true)
        ;
}

void assert_print(const char*, uint32_t, const char*, void*) {}

} // namespace

int main()
{
    using namespace cml::debug;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

    systick::enable((mcu::get_sysclk_frequency_hz() / 1000u) - 1, systick::Prescaler::_1, 0x9u);
    systick::register_tick_callback({ system_timer_update, nullptr });

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

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