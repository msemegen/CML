/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/ADC.hpp>
#include <hal/GPIO.hpp>
#include <hal/mcu.hpp>
#include <hal/systick.hpp>
#include <common/bit.hpp>
#include <common/frequency.hpp>

#include <hal/USART.hpp>
#include <utils/Console.hpp>
#include <utils/sleep.hpp>

namespace {

using namespace cml::common;
using namespace cml::hal;

int32 compute_temperature(const ADC::Calibration_data& a_calibration_data, uint32 measure)
{
    int32 temperature = 100 * ((measure * 330 / 300) - a_calibration_data.temperature_sensor_data_1);
    temperature /= (a_calibration_data.temperature_sensor_data_2 - a_calibration_data.temperature_sensor_data_1);

    return temperature + 30;
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

        systick::enable(0x0);

        mcu::disable_msi_clock();
        mcu::enable_dwt();

        ADC adc(ADC::Id::_1);

        GPIO gpio_c(GPIO::Id::c);
        gpio_c.enable();

        bool periph_ready = adc.enable(ADC::Resolution::_12_bit, { ADC::Synchronous_clock::Source::pclk,
                                                                   ADC::Synchronous_clock::Divider::_1 },
                                       20,
                                       1);

        if (true == periph_ready)
        {
            GPIO gpio_port_a(GPIO::Id::a);
            gpio_port_a.enable();

            Alternate_function_pin console_usart_TX_pin(&gpio_port_a, 2);
            Alternate_function_pin console_usart_RX_pin(&gpio_port_a, 3);

            console_usart_TX_pin.enable(usart_pin_config);
            console_usart_RX_pin.enable(usart_pin_config);

            USART console_usart(USART::Id::_2);
            bool preiph_ready = console_usart.enable(usart_config, usart_clock, 0x1u, 10);

            if (true == preiph_ready)
            {
                const ADC::Channel enabled_channels[] = { ADC::Channel::Id::temperature_sensor,
                                                          ADC::Channel::Sampling_time::_640_5_clock_cycles };

                adc.set_active_channels(enabled_channels, 1);

                Console console(&console_usart);
                console.enable();

                console.write_line("CML ADC sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

                while (true)
                {
                    uint16 r = 0;
                    adc.read_polling(&r, 1);

                    console.write_line("temp: %d, adc: %u\r", compute_temperature(adc.get_calibration_data(), r), r);
                    sleep::ms(10);
                }
            }
        }
    }

    while (true);
}