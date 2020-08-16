/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// cml
#include <cml/collection/Vector.hpp>
#include <cml/common/cstring.hpp>
#include <cml/hal/counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/systick.hpp>
#include <cml/utils/Command_line.hpp>
#include <cml/utils/Console.hpp>

namespace {

using namespace cml::collection;
using namespace cml::common;
using namespace cml::hal;
using namespace cml::hal::peripherals;
using namespace cml::utils;

void led_cli_callback(const Vector<Command_line::Callback::Parameter>& a_params, void* a_p_user_data)
{
    pin::Out* p_led_pin = reinterpret_cast<pin::Out*>(a_p_user_data);

    if (2 == a_params.get_length())
    {
        bool is_on = cstring::equals(a_params[1].a_p_value, "on", a_params[1].length);

        if (true == is_on)
        {
            p_led_pin->set_level(pin::Level::high);
        }
        else
        {
            bool is_off = cstring::equals(a_params[1].a_p_value, "off", a_params[1].length);

            if (true == is_off)
            {
                p_led_pin->set_level(pin::Level::low);
            }
        }
    }
}

void reset_callback(const Vector<Command_line::Callback::Parameter>& a_params, void* a_p_user_data)
{
    mcu::reset();
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

} // namespace

int main()
{
    using namespace cml;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi,
                    { mcu::Bus_prescalers::AHB::_1, mcu::Bus_prescalers::APB1::_1, mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 16u << 4u });

        USART::Config usart_config = { 115200u,
                                       USART::Oversampling::_16,
                                       USART::Stop_bits::_1,
                                       USART::Flow_control_flag::none,
                                       USART::Sampling_method::three_sample_bit,
                                       USART::Mode_flag::rx | USART::Mode_flag::tx };

        USART::Frame_format usart_frame_format { USART::Word_length::_8_bit, USART::Parity::none };

        USART::Clock usart_clock {
            USART::Clock::Source::sysclk,
            mcu::get_sysclk_frequency_hz(),
        };

        pin::af::Config usart_pin_config = { pin::Mode::push_pull, pin::Pull::up, pin::Speed::low, 0x7u };

        mcu::disable_msi_clock();

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        pin::af::enable(&gpio_port_a, 2, usart_pin_config);
        pin::af::enable(&gpio_port_a, 3, usart_pin_config);

        USART console_usart(USART::Id::_2);
        bool usart_ready = console_usart.enable(usart_config, usart_frame_format, usart_clock, 0x1u, 10);

        if (true == usart_ready)
        {
            pin::Out led_pin;
            pin::out::enable(&gpio_port_a, 5, { pin::Mode::push_pull, pin::Pull::down, pin::Speed::low }, &led_pin);

            Console console(
                { write_character, &console_usart }, { write_string, &console_usart }, { read_key, &console_usart });

            console.write_line("\nCML CLI sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            Command_line command_line({ write_character, &console_usart },
                                      { write_string, &console_usart },
                                      { read_key, &console_usart },
                                      "cmd > ",
                                      "Command not found");

            command_line.register_callback({ "led", led_cli_callback, &led_pin });
            command_line.register_callback({ "reset", reset_callback, nullptr });

            command_line.write_prompt();

            while (true)
            {
                command_line.update();
            }
        }
    }

    while (true)
        ;
}