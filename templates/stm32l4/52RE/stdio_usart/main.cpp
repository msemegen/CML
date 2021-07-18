/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include "syscalls.hpp"

// std
#include <cstdio>

// cml
#include <cml/hal/Systick.hpp>
#include <cml/hal/internal_flash.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/pwr.hpp>
#include <cml/hal/rcc.hpp>
#include <cml/hal/system_timer.hpp>

namespace {

using namespace cml::hal;

void system_timer_update(void*)
{
    system_timer::update();
}

void assert_halt(void*)
{
    mcu::halt();
    while (true)
        ;
}

void assert_print(const char* a_p_file, uint32_t a_line, const char* a_p_expression, void* a_p_user_data)
{
    printf("ASSERT failed: %s, LOC: %lu, expression: \"%s\"\n", a_p_file, a_line, a_p_expression);
}

} // namespace

int main()
{
    using namespace cml::debug;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;

    Systick systick;
    GPIO gpio_port_a(GPIO::Id::a);
    USART iostream(USART::Id::_2);

    internal_flash::set_latency(internal_flash::Latency::_4);
    pwr::set_voltage_scaling(pwr::Voltage_scaling::_1);

    rcc<mcu>::enable_clock(rcc<mcu>::Clock::hsi, rcc<mcu>::HSI_frequency::_16_MHz);
    rcc<mcu>::enable_clock(rcc<mcu>::Clock::pll,
                           rcc<mcu>::PLL_source::hsi,
                           rcc<mcu>::PLLM::_2,
                           { 20u,
                             { rcc<mcu>::PLL_config::R::Divider::_2, rcc<mcu>::PLL_config::Output::enabled },
                             { rcc<mcu>::PLL_config::Q::Divider::_2, rcc<mcu>::PLL_config::Output::disabled },
                             { rcc<mcu>::PLL_config::P::Divider::_7, rcc<mcu>::PLL_config::Output::disabled } });

    rcc<mcu>::set_sysclk_source(
        rcc<mcu>::SYSCLK_source::pll,
        { rcc<mcu>::Bus_prescalers::AHB::_1, rcc<mcu>::Bus_prescalers::APB1::_1, rcc<mcu>::Bus_prescalers::APB2::_1 });

    rcc<mcu>::disable_clock(rcc<mcu>::Clock::msi);

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

    systick.enable((rcc<mcu>::get_sysclk_frequency_hz() / 1000u) - 1, Systick::Prescaler::_1, 0x9u);
    systick.register_tick_callback({ system_timer_update, nullptr });

    rcc<GPIO>::enable(GPIO::Id::a, false);
    rcc<USART>::enable(USART::Id::_2, rcc<USART>::Clock_source::sysclk, false);

    gpio_port_a.enable();

    GPIO::Alternate_function::Config usart_pin_config = {
        GPIO::Mode::push_pull, GPIO::Pull::up, GPIO::Speed::high, 0x7u
    };

    gpio_port_a.p_alternate_function->enable(2u, usart_pin_config);
    gpio_port_a.p_alternate_function->enable(3u, usart_pin_config);

    bool iostream_ready = iostream.enable({ 115200,
                                            rcc<mcu>::get_sysclk_frequency_hz(),
                                            USART::Oversampling::_16,
                                            USART::Stop_bits::_1,
                                            USART::Flow_control_flag::none,
                                            USART::Sampling_method::three_sample_bit,
                                            USART::Mode_flag::tx | USART::Mode_flag::rx },
                                          { USART::Word_length::_8_bit, USART::Parity::none },
                                          0x1u,
                                          10u);

    if (true == iostream_ready)
    {
        initialize_syscalls(&iostream);

        printf("Sysclk source: ");

        rcc<mcu>::SYSCLK_source sysclk_source = rcc<mcu>::get_sysclk_source();
        switch (sysclk_source)
        {
            case rcc<mcu>::SYSCLK_source::msi: {
                printf("MSI\n");
            }
            break;

            case rcc<mcu>::SYSCLK_source::hsi: {
                printf("HSI\n");
            }
            break;

            case rcc<mcu>::SYSCLK_source::pll: {
                printf("PLL\n");
            }
            break;
        }

        printf("Clock frequency: ");

        if (rcc<mcu>::get_sysclk_frequency_hz() >= 1000000u)
        {
            printf("%lu MHz\n", rcc<mcu>::get_sysclk_frequency_hz() / 1000000u);
        }
        else if (rcc<mcu>::get_sysclk_frequency_hz() >= 1000u)
        {
            printf("%lu kHz\n", rcc<mcu>::get_sysclk_frequency_hz() / 1000u);
        }
        else
        {
            printf("%lu Hz\n", rcc<mcu>::get_sysclk_frequency_hz());
        }
    }

    setvbuf(stdout, nullptr, _IONBF, 0);

    while (true)
    {
        char name[20] = { 0 };

        setvbuf(stdin, name, _IOFBF, sizeof(name));
        printf("Enter your name: ");
        scanf("%20s", name);
        printf("Your name is: %s\nContinue [Y/N]?", name);
        setvbuf(stdin, nullptr, _IONBF, 0);

        while (true)
        {
            int32_t c = getc(stdin);

            if ('n' == c || 'N' == c)
            {
                printf("\nBye!\n");
                while (true)
                    ;
            }
            else if ('y' == c || 'Y' == c)
            {
                break;
            }
        }
        putc('\n', stdout);
    }

    while (true)
        ;

    return 0;
}