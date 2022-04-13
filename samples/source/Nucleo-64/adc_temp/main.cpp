/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdio>
#include <cstring>

// cml
#include <cml/debug/assertion.hpp>
#include <cml/hal/ADC.hpp>
#include <cml/hal/GPIO.hpp>
#include <cml/hal/Peripheral.hpp>
#include <cml/hal/Systick.hpp>
#include <cml/hal/USART.hpp>
#include <cml/hal/internal_flash.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/nvic.hpp>
#include <cml/hal/pwr.hpp>
#include <cml/hal/rcc.hpp>
#include <cml/utils/tick_counter.hpp>

// debug
#include <cml/hal/DMA.hpp>
#include <cml/hal/I2C.hpp>

namespace {
using namespace cml::hal;

void assert_halt(void*)
{
    mcu::halt();
    while (true)
        ;
}

void assert_print(const char*, uint32_t, const char*, void*) {}

} // namespace

int main()
{
    using namespace cml::hal;
    using namespace cml::debug;
    using namespace cml::utils;

    GPIO::Alternate_function::Enable_config usart_pin_config = {
        GPIO::Mode::push_pull, GPIO::Pull::up, GPIO::Speed::high, 0x7u
    };

    internal_flash::set_latency(internal_flash::Latency::_4);
    pwr::set(pwr::Voltage_scaling::_1);

    rcc<mcu>::HSI16::enable(rcc<mcu>::HSI16::Frequency::_16_MHz);
    rcc<mcu>::set_SYSCLK_source<rcc<mcu>::HSI16>(
        { rcc<mcu>::Bus_prescalers::AHB::_1, rcc<mcu>::Bus_prescalers::APB1::_1, rcc<mcu>::Bus_prescalers::APB2::_1 });
    rcc<mcu>::MSI::disable();

    nvic::set_config({ nvic::Config::Grouping::_4, 0x5u });

    Systick systick = Peripheral<Systick>::create();

    tick_counter::enable(&systick, { IRQ_config::Mode::enabled, 0x1u, 0x1u });
    assertion::enable({ assert_halt, nullptr }, { assert_print, nullptr }, assertion::Trap_enter_mode::enabled);

    mcu::set_DWT_mode(mcu::DWT_mode::enabled);

    GPIO gpio_port_a = Peripheral<GPIO, 1>::create();
    rcc<GPIO, 1>::enable(false);

    gpio_port_a.enable();

    gpio_port_a.alternate_function.enable(2u, usart_pin_config);
    gpio_port_a.alternate_function.enable(3u, usart_pin_config);

    USART usart = Peripheral<USART, 2>::create();
    rcc<USART, 2>::enable<rcc<USART, 2>::Clock_source::SYSCLK>(false);

    bool usart_ready = usart.enable({ 115200u,
                                      rcc<mcu>::get_HCLK_frequency_Hz(),
                                      USART::Enable_config::Oversampling::_16,
                                      USART::Enable_config::Stop_bits::_1,
                                      USART::Enable_config::Flow_control_flag::none,
                                      USART::Enable_config::Sampling_method::three_sample_bit,
                                      USART::Enable_config::Mode_flag::tx | USART::Enable_config::Mode_flag::rx },
                                    { USART::Frame_format::Word_length::_8_bit, USART::Frame_format::Parity::none },
                                    10u);

    if (true == usart_ready)
    {
        GPIO::Out::Pin led_pin;
        gpio_port_a.out.enable(5u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);

        rcc<ADC>::enable<rcc<ADC>::Clock_source::PCLK>(rcc<ADC>::PCLK_prescaler::_1, false);
        ADC adc = Peripheral<ADC, 1>::create();

        adc.enable(ADC::Resolution::_12_bit,
                   std::array { ADC::Channel { ADC::Channel::Id::temperature_sensor,
                                               ADC::Channel::Sampling_time::_640_5_clock_cycles } },
                   10_ms);

        while (true)
        {
            char string_buffer[64] = { 0 };
            std::uint16_t v        = 0;

            adc.polling.read<ADC::Mode::single>(&v, 1);

            std::int32_t temp_oC = static_cast<std::int32_t>(
                (30.0f / static_cast<float>(adc.get_calibration_data().temperature_sensor_data_2 -
                                            adc.get_calibration_data().temperature_sensor_data_1)) *
                    (v - adc.get_calibration_data().temperature_sensor_data_1) +
                30.0f);

            std::int32_t length =
                std::snprintf(string_buffer, sizeof(string_buffer), "Current temperature: %ld oC, %d\n", temp_oC, v);
            usart.polling.transmit(string_buffer, static_cast<std::size_t>(length));

            led_pin.toggle_level();
            tick_counter::delay(500_ms);
        }
    }

    while (true)
        ;

    return 0;
}