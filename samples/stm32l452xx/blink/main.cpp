/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/GPIO.hpp>
#include <hal/MCU.hpp>
#include <utils/sleep.hpp>

#include <common/bit.hpp>
#include <common/frequency.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;
    using namespace cml::utils;

    MCU::get_instance().enable_hsi_clock(MCU::Hsi_frequency::_16_MHz);
    MCU::get_instance().set_sysclk(MCU::Sysclk_source::hsi, { MCU::Bus_prescalers::AHB::_1,
                                                              MCU::Bus_prescalers::APB1::_1,
                                                              MCU::Bus_prescalers::APB2::_1 },
                                                              { 0x00000003, 15 << 4 });

    if (MCU::Sysclk_source::hsi == MCU::get_instance().get_sysclk_source())
    {
        Output_pin::Config led_pin_config =
        {
            Output_pin::Mode::push_pull,
            Output_pin::Pull::down,
            Output_pin::Speed::low
        };

        MCU::get_instance().disable_msi_clock();
        Systick::get_instance().enable(0x0);

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        Output_pin led_pin(&gpio_port_a, 5);
        led_pin.enable(led_pin_config);

        led_pin.set_level(Output_pin::Level::low);

        MCU::get_instance().enable_dwt();

        while (true)
        {
            sleep::ms(1000);
            led_pin.toggle_level();
        }
    }

    while (true);
}