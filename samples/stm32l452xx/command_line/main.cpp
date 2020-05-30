/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <collection/Vector.hpp>
#include <common/cstring.hpp>
#include <hal/GPIO.hpp>
#include <hal/mcu.hpp>
#include <hal/system_counter.hpp>
#include <hal/systick.hpp>
#include <hal/USART.hpp>
#include <utils/Command_line.hpp>
#include <utils/Console.hpp>

namespace
{

using namespace cml::collection;
using namespace cml::common;
using namespace cml::hal;
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
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 16u << 4u });

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
        systick::register_tick_callback({ system_counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        Alternate_function_pin console_usart_TX_pin(&gpio_port_a, 2);
        Alternate_function_pin console_usart_RX_pin(&gpio_port_a, 3);

        console_usart_TX_pin.enable(usart_pin_config);
        console_usart_RX_pin.enable(usart_pin_config);

        USART console_usart(USART::Id::_2);
        bool usart_ready = console_usart.enable(usart_config, usart_clock, 0x1u, 10);

        if (true == usart_ready)
        {
            Output_pin led_pin(&gpio_port_a, 5);
            led_pin.enable({ Output_pin::Mode::push_pull, Output_pin::Pull::down, Output_pin::Speed::low });

            Console console(&console_usart);
            console.write_line("\nCML CLI sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            Command_line command_line(&console_usart, "cmd > ", "Command not found");

            command_line.register_callback({ "led", led_cli_callback, &led_pin });
            command_line.enable();
            command_line.write_prompt();

            while (true)
            {
                command_line.update();
            }
        }
    }

    while (true);
}