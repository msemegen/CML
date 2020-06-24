/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/frequency.hpp>
#include <cml/hal/counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/systick.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/system/exti_controller.hpp>
#include <cml/utils/delay.hpp>

namespace {

using namespace cml::hal::peripherals;

bool exti_callback(Input_pin::Level, void* a_p_user_data)
{
    reinterpret_cast<Output_pin*>(a_p_user_data)->toggle_level();
    return true;
}

} // namespace ::

int main()
{
    using namespace cml;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::hal::system;
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
        mcu::enable_syscfg();

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        GPIO gpio_port_b(GPIO::Id::b);

        gpio_port_a.enable();
        gpio_port_b.enable();

        Output_pin led_pin(&gpio_port_b, 3u);
        led_pin.enable({ Output_pin::Mode::push_pull, Output_pin::Pull::down, Output_pin::Speed::low });
        led_pin.set_level(Output_pin::Level::low);

        Input_pin exti_in(&gpio_port_a, 9u);
        exti_in.enable(Input_pin::Pull::down);

        exti_controller::enable(0x5u);
        exti_controller::register_callback(&exti_in,
                                           exti_controller::Interrupt_mode::rising,
                                           { exti_callback, &led_pin });

        while (true){}
    }

    while (true);
}