/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/debug/assertion.hpp>
#include <cml/hal/GPIO.hpp>
#include <cml/hal/Peripheral.hpp>
#include <cml/hal/Systick.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/nvic.hpp>
#include <cml/hal/rcc.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/ms_tick_counter.hpp>

namespace {
using namespace cml::hal;

void assert_halt(void*)
{
    mcu::halt();
    while (true)
        ;
}

void gpio_interrupt_callback(uint32_t a_pin, void* a_p_user_data)
{
    GPIO::Out::Pin* p_led_pin = static_cast<GPIO::Out::Pin*>(a_p_user_data);
    p_led_pin->toggle_level();
}

void assert_print(const char*, uint32_t, const char*, void*) {}
} // namespace

int main()
{
    using namespace cml::hal;
    using namespace cml::debug;
    using namespace cml::utils;

    nvic::set_config({ nvic::Config::Grouping::_4, 0x5u });

    Systick systick = Peripheral<Systick>::create();

    systick.enable((rcc<mcu>::get_HCLK_frequency_Hz() / 1000u) - 1, Systick::Prescaler::_1);
    systick.interrupt.enable({ 0x1u, 0x1u });
    systick.interrupt.register_callback({ ms_tick_counter::update, nullptr });

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

    GPIO gpio_port_a = Peripheral<GPIO, 1>::create();
    GPIO gpio_port_c = Peripheral<GPIO, 3>::create();

    rcc<GPIO, 1>::enable(false);
    rcc<GPIO, 3>::enable(false);

    gpio_port_a.enable();
    gpio_port_c.enable();

    GPIO::Out::Pin led_pin;
    gpio_port_a.out.enable(5u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);
    gpio_port_c.in.enable(13u, GPIO::Pull::down);
    led_pin.set_level(GPIO::Level::high);

    rcc<mcu>::set_SYSCFG_active(true);
    GPIO::Interrupt gpio_interrupt { GPIO::Interrupt::Id::_10_15 };

    gpio_interrupt.enable({ gpio_interrupt_callback, &led_pin }, { 0x1u, 0x0 });
    gpio_interrupt.attach(gpio_port_c, 13u, GPIO::Interrupt::Trigger_flag::rising, GPIO::Interrupt::Mode::interrupt);

    while (true)
        ;

    return 0;
}