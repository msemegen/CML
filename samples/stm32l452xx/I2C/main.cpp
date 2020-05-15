/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/bit.hpp>
#include <common/frequency.hpp>
#include <hal/GPIO.hpp>
#include <hal/mcu.hpp>
#include <hal/systick.hpp>
#include <utils/delay.hpp>

//
#include <hal/stm32l452xx/I2C.hpp>

int main()
{
    using namespace cml::common;
    using namespace cml::hal;
    using namespace cml::utils;
    using namespace cml::hal::stm32l452xx;

    //filter enabled, filter disabled
    // { 0x10911E24,     0x10911E24 }, // 16 MHz


    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 10u << 4u });
        mcu::enable_syscfg();

        mcu::disable_msi_clock();
        systick::enable(9u);

        GPIO gpio_port_a(GPIO::Id::a);
        GPIO gpio_port_b(GPIO::Id::b);
        GPIO gpio_port_c(GPIO::Id::c);


        gpio_port_a.enable();
        gpio_port_b.enable();
        gpio_port_c.enable();

        Output_pin led_pin(&gpio_port_a, 5);
        led_pin.enable({ Output_pin::Mode::push_pull, Output_pin::Pull::down, Output_pin::Speed::low });

        led_pin.set_level(Output_pin::Level::low);

        I2C_master i2c_master(I2C_master::Id::_1);

        const uint8 data[] = { 0x6u, 0x0u };

        Alternate_function_pin::Config c;
        c.function = 0x4;
        c.mode = Output_pin::Mode::open_drain;
        c.pull = Output_pin::Pull::up;
        c.speed = Output_pin::Speed::low;



        Alternate_function_pin i2c_sda(&gpio_port_b, 8);
        Alternate_function_pin i2c_scl(&gpio_port_b, 9);

        Output_pin power(&gpio_port_c, 14);
        power.enable({Output_pin::Mode::push_pull, Output_pin::Pull::up, Output_pin::Speed::low});
        power.set_level(Output_pin::Level::high );

        i2c_sda.enable(c);
        i2c_scl.enable(c);

        //i2c_sda.set_level(Output_pin::Level::high);
        //i2c_scl.set_level(Output_pin::Level::low);

        i2c_master.enable({ false, 0x00200205, true }, I2C_master::Clock_source::sysclk, 0x1u);
        i2c_master.transmit_bytes_polling(16, data, sizeof(data));

        I2C_master::Bus_status err;

        while (true)
        {
            i2c_master.transmit_bytes_polling(16, data, sizeof(data), &err);

            delay::ms(100);
            led_pin.toggle_level();
            //i2c_scl.toggle_level();
            //i2c_sda.toggle_level();
        }
    }

    while (true);
}