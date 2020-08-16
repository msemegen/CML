/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// cml
#include <cml/bit.hpp>
#include <cml/frequency.hpp>
#include <cml/hal/counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/systick.hpp>
#include <cml/utils/delay.hpp>

int main()
{
    using namespace cml;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi,
                    { mcu::Bus_prescalers::AHB::_1, mcu::Bus_prescalers::APB1::_1, mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 10u << 4u });

        mcu::disable_msi_clock();

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        pin::Out led_pin;
        pin::out::enable(&gpio_port_a, 5, { pin::Mode::push_pull, pin::Pull::down, pin::Speed::low }, &led_pin);
        led_pin.set_level(pin::Level::low);

        while (true)
        {
            delay::ms(1000);
            led_pin.toggle_level();
        }
    }

    while (true)
        ;
}