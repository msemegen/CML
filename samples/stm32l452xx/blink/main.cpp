/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/gpio.hpp>
#include <hal/MCU.hpp>
#include <hal/Systick.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;

    MCU::get_instance().enable_HSI_clock(MCU::HSI_frequency::_16_MHz);
    MCU::get_instance().set_SYSCLK(MCU::SYSCLK_source::HSI, { MCU::Bus_prescalers::AHB::_1,
                                                              MCU::Bus_prescalers::APB1::_1,
                                                              MCU::Bus_prescalers::APB2::_1 },
                                                              { 0x00000003, 15 << 4 });

    if (MCU::SYSCLK_source::HSI == MCU::get_instance().get_SYSCLK_source())
    {
        Output_pin::Config led_pin_config =
        {
            Output_pin::Mode::push_pull,
            Output_pin::Pull::down,
            Output_pin::Speed::low
        };

        MCU::get_instance().disable_MSI_clock();
        Systick::get_instance().enable();

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        Output_pin led_pin(&gpio_port_a, 5);
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