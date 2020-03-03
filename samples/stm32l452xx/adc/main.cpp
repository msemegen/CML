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

int main()
{
    using namespace cml::common;
    using namespace cml::hal;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 16u << 4u });

        systick::enable(0x0);

        ADC adc_1(ADC::Id::_1);

        ADC::Clock clock;

        clock.source      = ADC::Clock::Source::synchronous;
        clock.synchronous = { ADC::Clock::Synchronous::Source::sysclk, ADC::Clock::Synchronous::Divider::_1 };

        mcu::enable_dwt();

        GPIO gpio_c(GPIO::Id::c);
        gpio_c.enable();

        bool ok = adc_1.enable(ADC::Resolution::_8_bit, clock, 10);

        if (true == ok)
        {
            Analog_pin channel_2(&gpio_c, 1);
            channel_2.enable(Input_pin::Pull::up);

            ADC::Channel channels[] = { ADC::Channel::Id::_1, ADC::Channel::Sampling_time::_640_5_clock_cycles };

            adc_1.set_active_channels(channels, 1);

            while (true)
            {
                uint16 data = 0;
                adc_1.read_polling(&data, 1);
                data = data;
            }
        }
    }


    while (true);
}