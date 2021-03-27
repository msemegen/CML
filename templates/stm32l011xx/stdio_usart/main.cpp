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
#include <cml/debug/assertion.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/hal/systick.hpp>

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
    printf("[%s, %lu]: \"%s\"\n", a_p_file, a_line, a_p_expression);
}

} // namespace

int main()
{
    using namespace cml::debug;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi,
                    { mcu::Bus_prescalers::AHB::_1, mcu::Bus_prescalers::APB1::_1, mcu::Bus_prescalers::APB2::_1 },
                    mcu::Voltage_scaling::_1);

    while (mcu::Sysclk_source::hsi != mcu::get_sysclk_source())
        ;

    mcu::disable_msi_clock();

    systick::enable((mcu::get_sysclk_frequency_hz() / 1000u) - 1, systick::Prescaler::_1, 0x9u);
    systick::register_tick_callback({ system_timer_update, nullptr });

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

    GPIO gpio_port_a(GPIO::Id::a);
    gpio_port_a.enable();

    GPIO::Alternate_function::Config usart_pin_config = {
        GPIO::Mode::push_pull, GPIO::Pull::up, GPIO::Speed::high, 0x4u
    };

    gpio_port_a.p_alternate_function->enable(2u, usart_pin_config);
    gpio_port_a.p_alternate_function->enable(15u, usart_pin_config);

    USART iostream(USART::Id::_2);

    bool iostream_ready = iostream.enable({ 115200,
                                            USART::Oversampling::_16,
                                            USART::Stop_bits::_1,
                                            USART::Flow_control_flag::none,
                                            USART::Sampling_method::three_sample_bit,
                                            USART::Mode_flag::tx | USART::Mode_flag::rx },
                                          { USART::Word_length::_8_bit, USART::Parity::none },
                                          { USART::Clock::Source::sysclk, mcu::get_sysclk_frequency_hz() },
                                          0x1u,
                                          10u);

    if (true == iostream_ready)
    {
        initialize_syscalls(&iostream);

        printf("Sysclk source: ");

        mcu::Sysclk_source sysclk_source = mcu::get_sysclk_source();
        switch (sysclk_source)
        {
            case mcu::Sysclk_source::msi: {
                printf("MSI\n");
            }
            break;

            case mcu::Sysclk_source::hsi: {
                printf("HSI\n");
            }
            break;

            case mcu::Sysclk_source::pll: {
                printf("PLL\n");
            }
            break;
        }

        printf("Clock frequency: ");

        if (mcu::get_sysclk_frequency_hz() >= 1000000u)
        {
            printf("%lu MHz\n", mcu::get_sysclk_frequency_hz() / 1000000u);
        }
        else if (mcu::get_sysclk_frequency_hz() >= 1000u)
        {
            printf("%lu kHz\n", mcu::get_sysclk_frequency_hz() / 1000u);
        }
        else
        {
            printf("%lu Hz\n", mcu::get_sysclk_frequency_hz());
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