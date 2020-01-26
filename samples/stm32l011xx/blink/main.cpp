/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <hal/GPIO.hpp>
#include <hal/MCU.hpp>
#include <hal/Systick.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;

    MCU::get_instance().enable_hsi_clock(MCU::Hsi_frequency::_16_MHz);
    MCU::get_instance().enable_pll(MCU::Pll_clock_source::hsi, { false,
                                                                 MCU::Pll_config::Multiplier::_4,
                                                                 MCU::Pll_config::Divider::_2 });
    MCU::get_instance().set_sysclk(MCU::Sysclk_source::pll, { MCU::Bus_prescalers::AHB::_1,
                                                              MCU::Bus_prescalers::APB1::_1,
                                                              MCU::Bus_prescalers::APB2::_1 });

    if (MCU::Sysclk_source::pll == MCU::get_instance().get_sysclk_source())
    {
        Output_pin::Config led_pin_config =
        {
            Output_pin::Mode::push_pull,
            Output_pin::Pull::down,
            Output_pin::Speed::low
        };

        MCU::get_instance().disable_msi_clock();
        Systick::get_instance().enable((1u << __NVIC_PRIO_BITS) - 1u);

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