/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/GPIO.hpp>
#include <hal/mcu.hpp>
#include <hal/system_counter.hpp>
#include <hal/systick.hpp>
#include <utils/delay.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::enable_pll({ mcu::Pll_config::Source::hsi,
                      false,
                      mcu::Pll_config::Multiplier::_4,
                      mcu::Pll_config::Divider::_2 });
    mcu::set_sysclk(mcu::Sysclk_source::pll, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::pll == mcu::get_sysclk_source())
    {
        mcu::disable_msi_clock();

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ system_counter::update, nullptr });

        GPIO gpio_port_b(GPIO::Id::b);
        gpio_port_b.enable();

        Output_pin led_pin(&gpio_port_b, 3);
        led_pin.enable({ Output_pin::Mode::push_pull, Output_pin::Pull::down, Output_pin::Speed::low });

        led_pin.set_level(Output_pin::Level::low);

        while (true)
        {
            delay::ms(1000);
            led_pin.toggle_level();
        }
    }

    while (true);
}