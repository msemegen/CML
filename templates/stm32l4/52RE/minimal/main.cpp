/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/debug/assertion.hpp>
#include <cml/hal/Systick.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/utils/delay.hpp>

#include <cml/hal/peripherals/Basic_timer.hpp>

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

    Systick systick;

    systick.enable((mcu::get_sysclk_frequency_hz() / 1000u) - 1, Systick::Prescaler::_1, 0x9u);
    systick.register_tick_callback({ system_timer_update, nullptr });

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

    GPIO gpio_port_a(GPIO::Id::a);

    rcc<GPIO>::enable(GPIO::Id::a, false);
    gpio_port_a.enable();

    GPIO::Out::Pin led_pin;
    gpio_port_a.p_out->enable(5u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);
    led_pin.set_level(GPIO::Level::low);

    rcc<Basic_timer>::enable(Basic_timer::Id::_6, true);
    Basic_timer timer(Basic_timer::Id::_6);

    timer.enable({ 10000u - 1u, (200u) - 1 }, 0x1u);
    timer.start();

    while (true)
    {
        while (false == timer.is_overload_event())
            ;
        led_pin.toggle_level();
        //delay::ms(500u);
    }
}