/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/debug/assertion.hpp>
#include <cml/hal/Factory.hpp>
#include <cml/hal/GPIO.hpp>
#include <cml/hal/Interrupt.hpp>
#include <cml/hal/Systick.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/nvic.hpp>
#include <cml/hal/rcc.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/utils/delay.hpp>

namespace {
using namespace cml::hal;

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
    using namespace cml::hal;
    using namespace cml::debug;
    using namespace cml::utils;

    nvic::set_config({ nvic::Config::Grouping::_4, 0x5u });
    
    Systick systick               = Factory<Systick>::create();
    Interrupt<Systick> systick_it = Factory<Interrupt<Systick>>::create(&systick);
    
    systick.enable((rcc<mcu>::get_SYSCLK_frequency_Hz() / 1000u) - 1, Systick::Prescaler::_1);
    systick_it.enable({ 0x1u, 0x1u });
    systick_it.register_callback({ system_timer::update, nullptr });
    
    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

    GPIO gpio_port_b = Factory<GPIO, 2>::create();

    rcc<GPIO, 2>::enable(false);
    gpio_port_b.enable();

    GPIO::Out::Pin led_pin;
    gpio_port_b.p_out->enable(3u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);
    led_pin.set_level(GPIO::Level::high);

    while (true)
    {
        led_pin.toggle_level();
        delay::ms(500u);
    }

    return 0;
}