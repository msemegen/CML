/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/ADC.hpp>
#include <hal/GPIO.hpp>
#include <hal/mcu.hpp>
#include <utils/sleep.hpp>

/* Temperature sensor calibration value address */ 
#define TEMP130_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007A)) 
#define VDD_CALIB ((uint16_t) (300))
#define VDD_APPLI ((uint16_t) (330)) 

cml::common::int32 ComputeTemperature(cml::common::uint32 measure)
{  
    cml::common::int32 temperature;
    temperature = ((measure * VDD_APPLI / VDD_CALIB) - (int32_t)*TEMP30_CAL_ADDR);
    temperature = temperature * (int32_t)(130 - 30); 
    temperature = temperature / (int32_t)(*TEMP130_CAL_ADDR - *TEMP30_CAL_ADDR); 
    temperature = temperature + 30;
    return(temperature);
}

int main()
{
    using namespace cml::common;
    using namespace cml::hal;
    using namespace cml::utils;

    mcu::enable_msi_clock(mcu::Msi_frequency::_4194_kHz);
    mcu::set_sysclk(mcu::Sysclk_source::msi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::msi == mcu::get_sysclk_source())
    {
        systick::enable(0x0u);

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        Analog_pin channel_0(&gpio_port_a, 0);
        channel_0.enable(Analog_pin::Pull::down);

        mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
        
        //Output_pin led_pin(&gpio_port_b, 3);
        //led_pin.enable({ Output_pin::Mode::push_pull, Output_pin::Pull::down, Output_pin::Speed::low });
        //
        //led_pin.set_level(Output_pin::Level::low);

        

        ADC adc(ADC::Id::_1);
        
        auto c = ADC::Channel::temperature_sensor;

        bool r = adc.enable(ADC::Resolution::_12_bit, { ADC::Asynchronous_clock::Source::hsi, ADC::Asynchronous_clock::Divider::_1 }, 30);

        if (true == r)
        {
            adc.set_active_channels(ADC::Sampling_time::_160_5_clock_cycles, &(c), 1);

            auto xy = adc.get_temperature_sensor_calibration_data_2();

            volatile uint32 c1 = *TEMP130_CAL_ADDR;
            volatile uint32 c2 = *TEMP30_CAL_ADDR;

            while (true)
            {
                uint16 r = 0;
                uint32 t = 0;

                adc.read_polling(&r, 1);
                t = ComputeTemperature(r);
                t = t;
                sleep::ms(10);
                //led_pin.toggle_level();
            }
        }
        else
        {
            while (true);
        }
    }

    while (true);
}