/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#ifndef CML_ASSERT
#define CML_ASSERT
#endif

#include <cml/collection/Array.hpp>
#include <cml/debug/assert.hpp>
#include <cml/hal/counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/systick.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/utils/Console.hpp>
#include <cml/utils/Logger.hpp>

namespace {

using namespace cml;
using namespace cml::hal;
using namespace cml::utils;

void print_assert(void* a_p_user_data,
                  const char* a_p_file,
                  uint32 a_line,
                  const char* a_p_expression)
{
    reinterpret_cast<Logger*>(a_p_user_data)->omg("%s : %u -> %s", a_p_file, a_line, a_p_expression);
}

void halt(void*)
{
    mcu::halt();
    while (true);
}

} // namespace ::

int main()
{
    using namespace cml::collection;
    using namespace cml::common;
    using namespace cml::debug;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        USART::Config usart_config =
        {
            115200u,
            USART::Oversampling::_16,
            USART::Word_length::_8_bits,
            USART::Stop_bits::_1,
            USART::Flow_control::none,
            USART::Parity::none,
            USART::Sampling_method::three_sample_bit
        };

        USART::Clock usart_clock
        {
            USART::Clock::Source::sysclk,
            mcu::get_sysclk_frequency_hz()
        };

        Alternate_function_pin::Config usart_pin_config =
        {
            Alternate_function_pin::Mode::push_pull,
            Alternate_function_pin::Pull::up,
            Alternate_function_pin::Speed::ultra,
            0x4u
        };

        mcu::disable_msi_clock();

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        Alternate_function_pin console_usart_tx_pin(&gpio_port_a, 2);
        Alternate_function_pin console_usart_rx_pin(&gpio_port_a, 15);

        console_usart_tx_pin.enable(usart_pin_config);
        console_usart_rx_pin.enable(usart_pin_config);

        USART console_usart(USART::Id::_2);
        console_usart.enable(usart_config, usart_clock, 0x1u, 10);

        Logger logger(&console_usart, true, true, true, true);
        logger.inf("CML assert sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

        assert::register_callback({ print_assert, &logger });
        assert::register_callback({ halt, nullptr });

        uint8 array_buffer[3];
        Array<uint8> array(array_buffer, sizeof(array_buffer));

        array[3] = 3;
    }

    while (true);
}