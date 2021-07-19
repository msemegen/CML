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
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
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

    systick.enable((rcc<mcu>::get_SYSCLK_frequency_Hz() / 1000u) - 1, Systick::Prescaler::_1, 0x9u);
    systick.register_tick_callback({ system_timer_update, nullptr });

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

    GPIO gpio_port_a(GPIO::Id::a);

    rcc<GPIO>::enable(GPIO::Id::a, false);
    gpio_port_a.enable();

    GPIO::Alternate_function::Config usart_pin_config = {
        GPIO::Mode::push_pull, GPIO::Pull::up, GPIO::Speed::high, 0x7u
    };

    gpio_port_a.p_alternate_function->enable(2u, usart_pin_config);
    gpio_port_a.p_alternate_function->enable(3u, usart_pin_config);

    USART iostream(USART::Id::_2);

    rcc<USART>::enable(USART::Id::_2, rcc<USART>::Clock_source::sysclk, false);
    bool iostream_ready = iostream.enable({ 115200,
                                            rcc<mcu>::get_SYSCLK_frequency_Hz(),
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

        rcc<mcu>::SYSCLK_source sysclk_source = rcc<mcu>::get_SYSCLK_source();
        switch (sysclk_source)
        {
            case rcc<mcu>::SYSCLK_source::MSI: {
                printf("MSI\n");
            }
            break;

            case rcc<mcu>::SYSCLK_source::HSI: {
                printf("HSI\n");
            }
            break;

            case rcc<mcu>::SYSCLK_source::PLL: {
                printf("PLL\n");
            }
            break;
        }

        printf("Clock frequency: ");

        if (rcc<mcu>::get_SYSCLK_frequency_Hz() >= 1000000u)
        {
            printf("%lu MHz\n", rcc<mcu>::get_SYSCLK_frequency_Hz() / 1000000u);
        }
        else if (rcc<mcu>::get_SYSCLK_frequency_Hz() >= 1000u)
        {
            printf("%lu kHz\n", rcc<mcu>::get_SYSCLK_frequency_Hz() / 1000u);
        }
        else
        {
            printf("%lu Hz\n", rcc<mcu>::get_SYSCLK_frequency_Hz());
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