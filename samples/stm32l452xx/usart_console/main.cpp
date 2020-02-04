/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/GPIO.hpp>
#include <hal/MCU.hpp>
#include <hal/Systick.hpp>
#include <hal/USART.hpp>
#include <utils/Console.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;
    using namespace cml::utils;

    MCU::get_instance().enable_hsi_clock(MCU::Hsi_frequency::_16_MHz);
    MCU::get_instance().set_sysclk(MCU::Sysclk_source::hsi, { MCU::Bus_prescalers::AHB::_1,
                                                              MCU::Bus_prescalers::APB1::_1,
                                                              MCU::Bus_prescalers::APB2::_1 },
                                                              { 0x3, 0xf0 });

    if (MCU::Sysclk_source::hsi == MCU::get_instance().get_sysclk_source())
    {
        USART::Config usart_config =
        {
            115200u,
            USART::Oversampling::_16,
            USART::Word_length::_8_bits,
            USART::Stop_bits::_1,
            USART::Flow_control::none,
            USART::Parity::none,
        };

        USART::Clock usart_clock
        {
            USART::Clock::Source::sysclk,
            SystemCoreClock
        };

        Alternate_function_pin::Config usart_pin_config =
        {
            Alternate_function_pin::Mode::push_pull,
            Alternate_function_pin::Pull::up,
            Alternate_function_pin::Speed::low,
            0x7u
        };

        MCU::get_instance().disable_msi_clock();
        Systick::get_instance().enable(0x0);

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        Alternate_function_pin console_usart_TX_pin(&gpio_port_a, 2);
        Alternate_function_pin console_usart_RX_pin(&gpio_port_a, 3);

        console_usart_TX_pin.enable(usart_pin_config);
        console_usart_RX_pin.enable(usart_pin_config);

        USART console_usart(USART::Id::_2);
        bool usart_ready = console_usart.enable(usart_config, usart_clock, 10);

        if (true == usart_ready)
        {
            Console console(&console_usart, Console::Input_mode::polling);

            while (true)
            {
                char c = console.read_key();
                console.write(c);
            }
        }
    }

    while (true);
}