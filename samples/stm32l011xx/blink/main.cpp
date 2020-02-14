/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <hal/GPIO.hpp>
#include <hal/mcu.hpp>
#include <utils/sleep.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::enable_pll(mcu::Pll_clock_source::hsi, { false,
                                                  mcu::Pll_config::Multiplier::_4,
                                                  mcu::Pll_config::Divider::_2 });
    mcu::set_sysclk(mcu::Sysclk_source::pll, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::pll == mcu::get_sysclk_source())
    {
        mcu::disable_msi_clock();
        Systick::get_instance().enable((1u << __NVIC_PRIO_BITS) - 1u);

        GPIO gpio_port_b(GPIO::Id::b);
        gpio_port_b.enable();

        Output_pin led_pin(&gpio_port_b, 3);
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