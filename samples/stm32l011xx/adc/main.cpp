/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// cml
#include <cml/hal/counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/ADC.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/systick.hpp>
#include <cml/utils/Console.hpp>
#include <cml/utils/delay.hpp>

namespace {

using namespace cml;
using namespace cml::hal::peripherals;

int32_t compute_temperature(const ADC::Calibration_data& a_calibration_data, uint32_t measure)
{
    int32_t temperature = 100 * ((measure * 330 / 300) - a_calibration_data.temperature_sensor_data_1);
    temperature /= (a_calibration_data.temperature_sensor_data_2 - a_calibration_data.temperature_sensor_data_1);

    return temperature + 30;
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

    mcu::enable_msi_clock(mcu::Msi_frequency::_4194_kHz);
    mcu::set_sysclk(mcu::Sysclk_source::msi,
                    { mcu::Bus_prescalers::AHB::_1, mcu::Bus_prescalers::APB1::_1, mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::msi == mcu::get_sysclk_source())
    {
        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });
        mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);

        ADC adc(ADC::Id::_1);

        bool periph_ready = adc.enable(ADC::Resolution::_12_bit,
                                       { ADC::Asynchronous_clock::Source::hsi, ADC::Asynchronous_clock::Divider::_1 },
                                       0x1u,
                                       10);

        if (true == periph_ready)
        {
            USART::Config usart_config = { 115200u,
                                           USART::Oversampling::_16,
                                           USART::Stop_bits::_1,
                                           USART::Flow_control_flag::none,
                                           USART::Sampling_method::three_sample_bit,
                                           USART::Mode_flag::tx };

            USART::Frame_format usart_frame_format { USART::Word_length::_8_bit, USART::Parity::none };

            USART::Clock usart_clock { USART::Clock::Source::sysclk, mcu::get_sysclk_frequency_hz() };

            pin::af::Config usart_pin_config = { pin::Mode::push_pull, pin::Pull::up, pin::Speed::ultra, 0x4u };

            GPIO gpio_port_a(GPIO::Id::a);
            gpio_port_a.enable();

            pin::af::enable(&gpio_port_a, 2u, usart_pin_config);
            pin::af::enable(&gpio_port_a, 15u, usart_pin_config);

            USART console_usart(USART::Id::_2);
            periph_ready = console_usart.enable(usart_config, usart_frame_format, usart_clock, 0x1u, 10);

            if (true == periph_ready)
            {
                const ADC::Channel enabled_channels[] = { ADC::Channel::temperature_sensor };
                adc.set_active_channels(ADC::Sampling_time::_160_5_clock_cycles, enabled_channels, 1);

                Console console({ write_character, &console_usart },
                                { write_string, &console_usart },
                                { read_key, &console_usart });

                console.write_line("CML ADC sample. CPU speed: %d MHz\n", mcu::get_sysclk_frequency_hz() / MHz(1));

                while (true)
                {
                    uint16_t r = 0;
                    adc.read_polling(&r, 1);

                    console.write_line("temp: %d, adc: %u\r", compute_temperature(adc.get_calibration_data(), r), r);
                    delay::ms(1000);
                }
            }
        }
    }

    while (true)
        ;
}