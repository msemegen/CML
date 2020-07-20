/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/collection/Vector.hpp>
#include <cml/common/cstring.hpp>
#include <cml/hal/counter.hpp>
#include <cml/hal/systick.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/utils/Command_line.hpp>

namespace
{

using namespace cml;
using namespace cml::collection;
using namespace cml::common;
using namespace cml::hal::peripherals;
using namespace cml::utils;

void led_cli_callback(const Vector<Command_line::Callback::Parameter>& a_params, void* a_p_user_data)
{
    Output_pin* p_led_pin = reinterpret_cast<Output_pin*>(a_p_user_data);

    if (2 == a_params.get_length())
    {
        bool is_on = cstring::equals(a_params[1].a_p_value, "on", a_params[1].length);

        if (true == is_on)
        {
            p_led_pin->set_level(Output_pin::Level::high);
        }
        else
        {
            bool is_off = cstring::equals(a_params[1].a_p_value, "off", a_params[1].length);

            if (true == is_off)
            {
                p_led_pin->set_level(Output_pin::Level::low);
            }
        }
    }
}

uint32_t write_character(char a_character, void* a_p_user_data)
{
    USART* p_console_usart = reinterpret_cast<USART*>(a_p_user_data);
    return p_console_usart->transmit_bytes_polling(&a_character, 1).data_length_in_words;
}

uint32_t write_string(const char* a_p_string, uint32_t a_length, void* a_p_user_data)
{
    USART* p_console_usart = reinterpret_cast<USART*>(a_p_user_data);
    return p_console_usart->transmit_bytes_polling(a_p_string, a_length).data_length_in_words;
}

uint32_t read_key(char* a_p_out, uint32_t a_length, void* a_p_user_data)
{
    USART* p_console_usart = reinterpret_cast<USART*>(a_p_user_data);
    return p_console_usart->receive_bytes_polling(a_p_out, a_length).data_length_in_words;
}

} // namespace ::

int main()
{
    using namespace cml::common;
    using namespace cml::hal;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        USART::Config usart_config =
        {
            115200u,
            USART::Oversampling::_16,
            USART::Stop_bits::_1,
            USART::Flow_control_flag::none,
            USART::Sampling_method::three_sample_bit,
            USART::Mode_flag::rx | USART::Mode_flag::tx
        };

        USART::Frame_format usart_frame_format
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
            Alternate_function_pin::Pull::up,
            Alternate_function_pin::Speed::low,
            0x4u
        };

        mcu::disable_msi_clock();

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        Alternate_function_pin console_usart_TX_pin(&gpio_port_a, 2);
        Alternate_function_pin console_usart_RX_pin(&gpio_port_a, 15);

        console_usart_TX_pin.enable(usart_pin_config);
        console_usart_RX_pin.enable(usart_pin_config);

        USART console_usart(USART::Id::_2);
        bool usart_ready = console_usart.enable(usart_config, usart_frame_format, usart_clock, 0x1u, 10);

        if (true == usart_ready)
        {
            GPIO gpio_port_b(GPIO::Id::b);
            gpio_port_b.enable();

            Output_pin led_pin(&gpio_port_b, 3);
            led_pin.enable({ Output_pin::Mode::push_pull, Output_pin::Pull::down, Output_pin::Speed::low });

            char preamble[36];
            cstring::format(preamble,
                            sizeof(preamble),
                            "\nCML CLI sample. CPU speed: %d MHz\n",
                            mcu::get_sysclk_frequency_hz() / MHz(1));

            console_usart.transmit_bytes_polling(preamble, sizeof(preamble));

            Command_line command_line({ write_character, &console_usart },
                                      { write_string,    &console_usart },
                                      { read_key,        &console_usart },
                                      "cmd > ",
                                      "Command not found");

            command_line.register_callback({ "led", led_cli_callback, &led_pin });
            command_line.write_prompt();

            while (true)
            {
                command_line.update();
            }
        }
    }

    while (true);
}