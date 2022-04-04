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

volatile bool b = false;

void dma_callback(DMA<>::Event_flag a_event, void* a_p_user_data)
{
    // ADC* p_adc = reinterpret_cast<ADC*>(a_p_user_data);
    // auto x     = 0;
    // x          = x;
    b = true;
}
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
    pwr::set_voltage_scaling(pwr::Voltage_scaling::_1);

    rcc<mcu>::enable_clock<rcc<mcu>::Clock::HSI>(rcc<mcu>::HSI_frequency::_16_MHz);
    rcc<mcu>::enable_clock<rcc<mcu>::Clock::PLL>(rcc<mcu>::PLL_source::HSI, rcc<mcu>::PLLM::_2, {
        20u, { rcc<mcu>::PLL_config::R::Divider::_2, rcc<mcu>::PLL_config::Output::enabled },
            { rcc<mcu>::PLL_config::Q::Divider::_2, rcc<mcu>::PLL_config::Output::disabled },
#if defined(SOC_PLL_P_PRESENT)
        {
            rcc<mcu>::PLL_config::P::Divider::_7, rcc<mcu>::PLL_config::Output::disabled
        }
#endif
    });

    rcc<mcu>::set_SYSCLK_source(
        rcc<mcu>::SYSCLK_source::PLL,
        { rcc<mcu>::Bus_prescalers::AHB::_1, rcc<mcu>::Bus_prescalers::APB1::_1, rcc<mcu>::Bus_prescalers::APB2::_1 });

    rcc<mcu>::disable_clock(rcc<mcu>::Clock::MSI);
    nvic::set_config({ nvic::Config::Grouping::_4, 0x5u });

    Systick systick = Peripheral<Systick>::create();

    tick_counter::enable(&systick, { IRQ_config::Mode::enabled, 0x1u, 0x1u });
    assertion::enable({ assert_halt, nullptr }, { assert_print, nullptr });

    mcu::set_DWT_active(true);

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
                                    { USART::Frame_config::Word_length::_8_bit, USART::Frame_config::Parity::none },
                                    10u);

    if (true == usart_ready)
    {
        GPIO::Out::Pin led_pin;
        gpio_port_a.out.enable(5u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);

        GPIO gpio_port_c = Peripheral<GPIO, 3>::create();
        rcc<GPIO, 3>::enable(false);

        gpio_port_c.analog.enable(0x0u, GPIO::Pull::none);

        rcc<ADC>::enable<rcc<ADC>::Clock_source::PCLK>(rcc<ADC>::PCLK_prescaler::_1, false);
        ADC adc = Peripheral<ADC, 1>::create();

        adc.enable(
            ADC::Resolution::_6_bit,
            std::array {
                ADC::Channel { ADC::Channel::Id::temperature_sensor, ADC::Channel::Sampling_time::_640_5_clock_cycles },
                ADC::Channel { ADC::Channel::Id::_1, ADC::Channel::Sampling_time::_640_5_clock_cycles } },
            10_ms);

        rcc<DMA<>, 1>::enable();
        DMA<ADC> adc_dma = Peripheral<ADC, 1, DMA<>, 1>::create();

        std::uint16_t v[2] = { 0x0, 0x0u };
        adc_dma.read_start<ADC::Mode::continuous>(DMA<>::Priority::low,
                                                  DMA<>::Mode::single,
                                                  &v,
                                                  sizeof(v),
                                                  IRQ_config { IRQ_config::Mode::enabled, 0x0u, 0x0u },
                                                  { dma_callback, &adc },
                                                  DMA<>::Event_flag::transfer_complete);

        while (true)
        {
            char string_buffer[32] = { 0 };

            while (false == b)
                ;

            std::int32_t length = std::snprintf(string_buffer, sizeof(string_buffer), "%d %d\n", v[0], v[1]);
            usart.polling.transmit(string_buffer, static_cast<std::size_t>(length));

            b = false;

            // adc.polling.read<ADC::Mode::single>(&v, 1);

            // std::int32_t temp_oC = static_cast<std::int32_t>(
            //     (30.0f / static_cast<float>(adc.get_calibration_data().temperature_sensor_data_2 -
            //                                 adc.get_calibration_data().temperature_sensor_data_1)) *
            //         (v - adc.get_calibration_data().temperature_sensor_data_1) +
            //     30.0f);
            //
            // std::int32_t length =
            //     std::snprintf(string_buffer, sizeof(string_buffer), "Current temperature: %ld oC\n", temp_oC);
            // usart.polling.transmit(string_buffer, static_cast<std::size_t>(length));

            // std::int32_t length = std::snprintf(string_buffer, sizeof(string_buffer), "%d\n", v);
            // usart.polling.transmit(string_buffer, static_cast<std::size_t>(length));

            // led_pin.toggle_level();
            // tick_counter::delay(500_ms);
            b = false;
        }
    }

    while (true)
        ;

    return 0;
}