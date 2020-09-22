/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// cml
#include <cml/frequency.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>

namespace {

using namespace cml::hal::peripherals;

bool exti_callback(pin::Level, void* a_p_user_data)
{
    reinterpret_cast<pin::Out*>(a_p_user_data)->toggle_level();
    return true;
}

} // namespace

int main()
{
    using namespace cml;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi,
                    { mcu::Bus_prescalers::AHB::_1, mcu::Bus_prescalers::APB1::_1, mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 10u << 4u });
        mcu::enable_syscfg();

        mcu::disable_msi_clock();

        GPIO gpio_port_a(GPIO::Id::a);
        GPIO gpio_port_c(GPIO::Id::c);

        gpio_port_a.enable();
        gpio_port_c.enable();

        pin::Out led_pin;
        pin::out::enable(&gpio_port_a, 5u, { pin::Mode::push_pull, pin::Pull::down, pin::Speed::low }, &led_pin);
        led_pin.set_level(pin::Level::low);

        pin::in::enable_interrupt_line(pin::in::Interrupt_line::exti_10_15, 0x5u);
        pin::in::enable_interrupt(
            &gpio_port_c, 13u, pin::Pull::none, pin::in::Interrupt_mode::rising, { exti_callback, &led_pin });

        while (true)
            ;
    }

    while (true)
        ;
}