/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/debug/assertion.hpp>
#include <cml/hal/EXTI.hpp>
#include <cml/hal/Systick.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/nvic.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/rcc.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/utils/delay.hpp>

namespace {

using namespace cml::hal;
using namespace cml::hal::peripherals;

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

void exti_callback(uint32_t a_pin, void* a_p_user_data)
{
    GPIO::Out::Pin* p_led_pin = static_cast<GPIO::Out::Pin*>(a_p_user_data);
    p_led_pin->toggle_level();
}

} // namespace

int main()
{
    using namespace cml::debug;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

    Systick systick;

    nvic::set_config({ nvic::Config::Grouping::_4, 10u << 4u });

    systick.enable((rcc<mcu>::get_sysclk_frequency_hz() / 1000u) - 1, Systick::Prescaler::_1, 0x9u);
    systick.register_tick_callback({ system_timer_update, nullptr });

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

    GPIO gpio_port_a(GPIO::Id::a);
    GPIO gpio_port_c(GPIO::Id::c);

    rcc<GPIO>::enable(GPIO::Id::a, false);
    rcc<GPIO>::enable(GPIO::Id::c, false);

    gpio_port_a.enable();
    gpio_port_c.enable();

    GPIO::Out::Pin led_pin;
    gpio_port_a.p_out->enable(5u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);
    led_pin.set_level(GPIO::Level::low);

    gpio_port_c.p_in->enable(13u, GPIO::Pull::down);

    rcc<mcu>::set_syscfg_mode(rcc<mcu>::SYSCFG_mode::enabled);

    EXTI<GPIO> exti(EXTI<GPIO>::Id::_10_15);
    exti.enable({ exti_callback, &led_pin }, 0x1u);
    exti.attach(gpio_port_c, 13u, EXTI<GPIO>::Trigger_flag::rising);

    while (true)
        ;

    return 0;
}