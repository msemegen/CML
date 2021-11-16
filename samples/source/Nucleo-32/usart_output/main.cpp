/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/debug/assertion.hpp>
#include <cml/hal/Factory.hpp>
#include <cml/hal/GPIO.hpp>
#include <cml/hal/Interrupt.hpp>
#include <cml/hal/Polling.hpp>
#include <cml/hal/Systick.hpp>
#include <cml/hal/USART.hpp>
#include <cml/hal/internal_flash.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/nvic.hpp>
#include <cml/hal/pwr.hpp>
#include <cml/hal/rcc.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/utils/delay.hpp>

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
    pwr::set_voltage_scaling(pwr::Voltage_scaling::_1);

    rcc<mcu>::enable_clock<rcc<mcu>::Clock::HSI>(rcc<mcu>::HSI_frequency::_16_MHz);
    rcc<mcu>::enable_clock<rcc<mcu>::Clock::PLL>(rcc<mcu>::PLL_source::HSI, rcc<mcu>::PLLM::_2, {
        20u, { rcc<mcu>::PLL_config::R::Divider::_2, rcc<mcu>::PLL_config::Output::enabled },
            { rcc<mcu>::PLL_config::Q::Divider::_2, rcc<mcu>::PLL_config::Output::disabled },
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
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

    Systick systick               = Factory<Systick>::create();
    Interrupt<Systick> systick_it = Factory<Interrupt<Systick>>::create(&systick);

    systick.enable((rcc<mcu>::get_SYSCLK_frequency_Hz() / 1000u) - 1, Systick::Prescaler::_1);
    systick_it.enable({ 0x1u, 0x1u });
    systick_it.register_callback({ system_timer::update, nullptr });

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

    GPIO gpio_port_a = Factory<GPIO, 1>::create();
    GPIO gpio_port_b = Factory<GPIO, 2>::create();

    rcc<GPIO, 1>::enable(false);
    rcc<GPIO, 2>::enable(false);
    gpio_port_a.enable();
    gpio_port_b.enable();

    gpio_port_a.p_alternate_function->enable(2u, usart_pin_config);
    gpio_port_a.p_alternate_function->enable(3u, usart_pin_config);

    USART usart = Factory<USART, 2>::create();
    rcc<USART, 2>::enable<rcc<USART, 2>::Clock_source::SYSCLK>(false);

    bool usart_ready = usart.enable({ 115200u,
                                      rcc<mcu>::get_SYSCLK_frequency_Hz(),
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
        gpio_port_b.p_out->enable(3u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);

        Polling<USART> iostream = Factory<Polling<USART>, 2>::create(&usart);

        while (true)
        {
            iostream.transmit("Hello world\n", 12u);
            led_pin.toggle_level();
            delay::ms(500);
        }
    }

    while (true)
        ;

    return 0;
}