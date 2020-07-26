/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
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
using namespace cml::hal::peripherals;
using namespace cml::utils;

void print_assert(void* a_p_user_data,
                  const char* a_p_file,
                  uint32_t a_line,
                  const char* a_p_expression)
{
    reinterpret_cast<Logger*>(a_p_user_data)->omg("%s : %u -> %s\n", a_p_file, a_line, a_p_expression);
}

void halt(void*)
{
    mcu::halt();
    while (true);
}

uint32_t write_string(const char* a_p_string, uint32_t a_length, void* a_p_user_data)
{
    USART* p_console_usart = reinterpret_cast<USART*>(a_p_user_data);
    return p_console_usart->transmit_bytes_polling(a_p_string, a_length).data_length_in_words;
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
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 16u << 4u });

        USART::Config usart_config =
        {
            115200u,
            USART::Oversampling::_16,
            USART::Stop_bits::_1,
            USART::Flow_control_flag::none,
            USART::Sampling_method::three_sample_bit,
            USART::Mode_flag::tx
        };

        USART::Frame_format usart_frame_format
        {
            USART::Word_length::_8_bit,
            USART::Parity::none
        };

        USART::Clock usart_clock
        {
            USART::Clock::Source::sysclk,
            mcu::get_sysclk_frequency_hz(),
        };

        pin::af::Config usart_pin_config =
        {
            pin::Mode::push_pull,
            pin::Pull::up,
            pin::Speed::low,
            0x7u
        };

        mcu::disable_msi_clock();
        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        pin::af::enable(&gpio_port_a, 2, usart_pin_config);
        pin::af::enable(&gpio_port_a, 3, usart_pin_config);

        USART console_usart(USART::Id::_2);
        console_usart.enable(usart_config, usart_frame_format, usart_clock, 0x1u, 10);

        Logger logger({ write_string, &console_usart }, true, true, true, true);
        logger.inf("CML assert sample. CPU speed: %u MHz\n", mcu::get_sysclk_frequency_hz() / MHz(1));

        assert::register_print({ print_assert, &logger });
        assert::register_halt({ halt, nullptr });

        uint8_t array_buffer[3];
        Array<uint8_t> array(array_buffer, sizeof(array_buffer));

        array[3] = 3;
    }

    while (true);
}