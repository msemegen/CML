/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdio>

// externals
#include <stm32l4xx.h>

// cml
#include <cml/debug/assertion.hpp>
#include <cml/hal/Systick.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/rcc.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/utils/delay.hpp>

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
    using namespace cml::utils;

    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    Systick systick;

    systick.enable((rcc<mcu>::get_sysclk_frequency_hz() / 1000u) - 1, Systick::Prescaler::_1, 0x9u);
    systick.register_tick_callback({ system_timer_update, nullptr });

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

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

    uint32_t i = 0;
    while (true)
    {
        printf("T: %lu\n", i++);
        delay::s(1u);
    }

    while (true)
        ;

    return 0;
}