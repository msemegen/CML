/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/gpio.hpp>
#include <hal/mcu.hpp>
#include <hal/systick.hpp>
#include <hal/usart.hpp>
#include <utils/console.hpp>
#include <utils/sleep.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;
    using namespace cml::utils;

    c_mcu::get_instance().enable_hsi_clock(c_mcu::e_hsi_frequency::_16_MHz);
    c_mcu::get_instance().set_sysclk(c_mcu::e_sysclk_source::hsi, { c_mcu::s_bus_prescalers::e_ahb::_1,
                                                                    c_mcu::s_bus_prescalers::e_apb1::_1,
                                                                    c_mcu::s_bus_prescalers::e_apb2::_1 });

    if (c_mcu::e_sysclk_source::hsi == c_mcu::get_instance().get_sysclk_source())
    {
        c_usart::s_config usart_config =
        {
            c_usart::e_baud_rate::_115200,
            c_usart::e_oversampling::_16,
            c_usart::e_word_length::_8_bits,
            c_usart::e_stop_bits::_1,
            c_usart::e_flow_control::none,
            c_usart::e_parity::none,
        };

        c_usart::s_clock usart_clock
        {
            c_usart::s_clock::e_source::sysclk,
            SystemCoreClock
        };

        c_alternate_function_pin::s_config usart_pin_config =
        {
            c_gpio::e_mode::push_pull,
            c_gpio::e_pull::up,
            c_gpio::e_speed::ultra,
            0x4u
        };

        c_mcu::get_instance().disable_msi_clock();
        c_systick::get_instance().enable();

        c_gpio gpio_port_a(c_gpio::e_periph::a);
        gpio_port_a.enable();

        c_alternate_function_pin console_usart_tx_pin(&gpio_port_a, 2);
        c_alternate_function_pin console_usart_rx_pin(&gpio_port_a, 15);

        console_usart_tx_pin.enable(usart_pin_config);
        console_usart_rx_pin.enable(usart_pin_config);

        c_usart console_usart(c_usart::e_periph::_2);
        bool usart_ready = console_usart.enable(usart_config, usart_clock, 10);

        if (true == usart_ready)
        {
            c_console console(&console_usart);

            while (true)
            {
                console.read_key(true);
            }
        }
    }

    while (true);
}