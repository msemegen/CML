/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <hal/gpio.hpp>
#include <hal/mcu.hpp>
#include <hal/systick.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;

    c_mcu::get_instance().enable_hsi_clock(c_mcu::e_hsi_frequency::_16_MHz);
    c_mcu::get_instance().enable_pll(c_mcu::e_pll_clock_source::hsi, { false,
                                                                       c_mcu::s_pll_config::e_pll_multiplier::_4,
                                                                       c_mcu::s_pll_config::e_pll_divider::_2 });
    c_mcu::get_instance().set_sysclk(c_mcu::e_sysclk_source::pll, { c_mcu::s_bus_prescalers::e_ahb::_1,
                                                                    c_mcu::s_bus_prescalers::e_apb1::_1,
                                                                    c_mcu::s_bus_prescalers::e_apb2::_1 });

    if (c_mcu::e_sysclk_source::pll == c_mcu::get_instance().get_sysclk_source())
    {
        c_output_pin::s_config led_pin_config =
        {
            c_gpio::e_mode::push_pull,
            c_gpio::e_pull::down,
            c_gpio::e_speed::low
        };

        c_mcu::get_instance().disable_msi_clock();
        c_systick::get_instance().enable();

        c_gpio gpio_port_b(c_gpio::e_periph::b);
        gpio_port_b.enable();

        c_output_pin led_pin(&gpio_port_b, 3);
        led_pin.enable(led_pin_config);

        led_pin.set_level(c_gpio::e_level::low);

        time_tick start = c_systick::get_instance().get_counter();

        while (true)
        {
            if (c_systick::get_instance().get_counter() - start >= 500u)
            {
                led_pin.toggle_level();
                start = c_systick::get_instance().get_counter();
            }
        }
    }

    while (true);
}