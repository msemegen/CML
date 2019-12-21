/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/gpio.hpp>
#include <hal/mcu.hpp>
#include <hal/systick.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;

    c_output_pin::s_config led_pin_config =
    {
        c_gpio::e_mode::push_pull,
        c_gpio::e_pull::down,
        c_gpio::e_speed::low
    };

    c_mcu::get_instance().enable_hsi_clock(c_mcu::e_hsi_frequency::_16_MHz);
    c_mcu::get_instance().set_sysclk(c_mcu::e_sysclk_source::hsi, { c_mcu::s_bus_prescalers::e_ahb::_1,
                                                                    c_mcu::s_bus_prescalers::e_apb1::_1,
                                                                    c_mcu::s_bus_prescalers::e_apb2::_1 },
                                                                    { 0x00000003, 15 << 4 });
    c_mcu::get_instance().disable_msi_clock();

    if (c_mcu::e_sysclk_source::hsi == c_mcu::get_instance().get_sysclk_source())
    {
        c_systick::get_instance().enable();

        c_gpio gpio_port_a(c_gpio::e_periph::a);
        gpio_port_a.enable();

        c_output_pin led_pin(&gpio_port_a, 5);
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