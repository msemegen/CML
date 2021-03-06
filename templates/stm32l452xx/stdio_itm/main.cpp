/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdio>

// externals
#include <stm32l452xx.h>

// cml
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/hal/systick.hpp>
#include <cml/utils/delay.hpp>

namespace {
using namespace cml::hal;
void system_timer_update(void*)
{
    system_timer::update();
}

} // namespace

int main()
{
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    systick::enable((mcu::get_sysclk_frequency_hz() / 1000u) - 1, systick::Prescaler::_1, 0x9u);
    systick::register_tick_callback({ system_timer_update, nullptr });

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