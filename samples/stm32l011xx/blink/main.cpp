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

    MCU::get_instance().enable_HSI_clock(MCU::HSI_frequency::_16_MHz);
    MCU::get_instance().enable_PLL(MCU::PLL_clock_source::hsi, { false,
                                                                 MCU::PLL_config::Multiplier::_4,
                                                                 MCU::PLL_config::Divider::_2 });
    MCU::get_instance().set_SYSCLK(MCU::SYSCLK_source::PLL, { MCU::Bus_prescalers::AHB::_1,
                                                              MCU::Bus_prescalers::APB1::_1,
                                                              MCU::Bus_prescalers::APB2::_1 });

    if (MCU::SYSCLK_source::PLL == MCU::get_instance().get_SYSCLK_source())
    {
        Output_pin::Config led_pin_config =
        {
            Output_pin::Mode::push_pull,
            Output_pin::Pull::down,
            Output_pin::Speed::low
        };

        MCU::get_instance().disable_MSI_clock();
        Systick::get_instance().enable();

        GPIO gpio_port_b(GPIO::Id::b);
        gpio_port_b.enable();

        Output_pin led_pin(&gpio_port_b, 3);
        led_pin.enable(led_pin_config);

        led_pin.set_level(Output_pin::Level::low);

        time_tick start = Systick::get_instance().get_counter();

        while (true)
        {
            if (Systick::get_instance().get_counter() - start >= 500u)
            {
                led_pin.toggle_level();
                start = Systick::get_instance().get_counter();
            }
        }
    }

    while (true);
}