/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/hal/counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/systick.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/utils/Buffered_console.hpp>
#include <cml/utils/delay.hpp>

int main()
{
    using namespace cml;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 16u << 4u });

        USART::Config usart_config =
        {
            115200u,
            USART::Oversampling::_16,
            USART::Stop_bits::_1,
            USART::Flow_control_flag::none,
            USART::Parity::none,
            USART::Sampling_method::three_sample_bit
        };

        USART::Clock usart_clock
        {
            USART::Clock::Source::sysclk,
            mcu::get_sysclk_frequency_hz(),
        };

        Alternate_function_pin::Config usart_pin_config =
        {
            Alternate_function_pin::Mode::push_pull,
            Alternate_function_pin::Pull::up,
            Alternate_function_pin::Speed::low,
            0x7u
        };

        mcu::disable_msi_clock();
        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        Alternate_function_pin console_usart_tx_pin(&gpio_port_a, 2);
        Alternate_function_pin console_usart_rx_pin(&gpio_port_a, 3);

        console_usart_tx_pin.enable(usart_pin_config);
        console_usart_rx_pin.enable(usart_pin_config);

        USART console_usart(USART::Id::_2);
        bool usart_ready = console_usart.enable(usart_config, usart_clock, 0x1u, 10);

        if (true == usart_ready)
        {
            Buffered_console console(&console_usart);

            console.enable();
            console.write_line("CML Console sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            while (true)
            {
                char c;
                console.read_key(&c);
                console.write(c);
            }
        }
    }

    while (true);
}