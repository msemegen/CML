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
            USART::Sampling_method::three_sample_bit
        };

        USART::Frame_format frame_format_1 =
        {
            USART::Word_length::_9_bit,
            USART::Parity::none
        };

        USART::Frame_format frame_format_2 =
        {
            USART::Word_length::_8_bit,
            USART::Parity::none
        };


        USART::Clock usart_clock
        {
            USART::Clock::Source::sysclk,
            mcu::get_sysclk_frequency_hz(),
        };

        Alternate_function_pin::Config usart_pin_config =
        {
            Alternate_function_pin::Mode::push_pull,
            Alternate_function_pin::Pull::down,
            Alternate_function_pin::Speed::low,
            0x7u
        };

        mcu::disable_msi_clock();
        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        GPIO gpio_port_c(GPIO::Id::c);

        gpio_port_a.enable();
        gpio_port_c.enable();

        Alternate_function_pin interlink_usart_tx_pin(&gpio_port_c, 4);
        Alternate_function_pin interlink_usart_rx_pin(&gpio_port_c, 5);

        interlink_usart_tx_pin.enable(usart_pin_config);
        interlink_usart_rx_pin.enable(usart_pin_config);


        Alternate_function_pin console_usart_tx_pin(&gpio_port_a, 2);
        Alternate_function_pin console_usart_rx_pin(&gpio_port_a, 3);

        console_usart_tx_pin.enable(usart_pin_config);
        console_usart_rx_pin.enable(usart_pin_config);

        USART console_usart(USART::Id::_2);
        console_usart.enable(usart_config, frame_format_2, usart_clock, 0x1u, 10);

        USART interlink_usart(USART::Id::_3);
        bool usart_ready = interlink_usart.enable(usart_config, frame_format_1, usart_clock, 0x1u, 10);

        if (true == usart_ready)
        {
            Buffered_console console(&console_usart);
            console.write_line("CML Console sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            const uint16 addr[] = { 0xC1, 0xC2, 0xC };

            while (true)
            {
                //char c = console.read_key();
                //console.write(c);

               // for (uint8 i = 0; i < 3; i++)
                {
                    const uint16 data[] = {0x10Fu, 0x1FFu, 0xFu };
                    //uint32 data = cml::numeric_traits<uint32>::get_max();
                    auto[f, words] = interlink_usart.transmit_bytes_polling(data, sizeof(data) / sizeof(data[0]));

                    if (f != USART::Bus_status_flag::ok)
                    {
                        console.write_line("Error! %u", static_cast<uint32>(f));
                    }
                    else
                    {
                        console.write_line("Data sent: %u", words);
                    }

                    //if (USART::Bus_status_flag::ok != f)
                    //{
                    //    x = 3;
                    //    x = x;
                    //}

                    delay::ms(1);
                }
            }
        }
    }

    while (true);
}