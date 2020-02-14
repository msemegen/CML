/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/GPIO.hpp>
#include <hal/mcu.hpp>
#include <utils/sleep.hpp>

#include <common/bit.hpp>
#include <common/frequency.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 },
                                               { 0x00000003, 15 << 4 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::disable_msi_clock();
        Systick::get_instance().enable(0x0);

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        Output_pin led_pin(&gpio_port_a, 5);
        led_pin.enable({ Output_pin::Mode::push_pull, Output_pin::Pull::down, Output_pin::Speed::low });

        led_pin.set_level(Output_pin::Level::low);

        while (true)
        {
            sleep::ms(1000);
            led_pin.toggle_level();
        }
    }

    while (true);
}