/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// cml
#include <cml/debug/assert.hpp>
#include <cml/frequency.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/ADC.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/hal/systick.hpp>
#include <cml/utils/Logger.hpp>
#include <cml/utils/delay.hpp>

namespace {

using namespace cml::hal;
using namespace cml::hal::peripherals;
using namespace cml::utils;

void assert_mcu_halt(void*)
{
    mcu::halt();
}

void assert_print(const char* a_p_file, const char* a_p_line, const char* a_p_expression, void* a_p_user_data)
{
    USART* p_usart = reinterpret_cast<USART*>(a_p_user_data);

    auto print = [&](const char* a_p_string) -> void {
        while ('\0' != (*a_p_string))
        {
            p_usart->transmit_word(*a_p_string);
            a_p_string++;
        }
    };

    print("\e[01;31m[ASSERT]\e[0m ");
    print(a_p_file);
    print(" : ");
    print(a_p_line);
    print(" -> ");
    print(a_p_expression);
    print("\n");
}

uint32_t write_string(const char* a_p_string, uint32_t a_length, void* a_p_user_data)
{
    return reinterpret_cast<USART*>(a_p_user_data)->transmit_bytes_polling(a_p_string, a_length).data_length_in_words;
}

void pre_sysclk_freq_change(void*) {}
void post_sysclk_freq_change(void*) {}

const char* sysclk_source_to_cstring(mcu::Sysclk_source a_source)
{
    switch (a_source)
    {
        case mcu::Sysclk_source::msi:
            return "MSI";

        case mcu::Sysclk_source::hsi:
            return "HSI";

        case mcu::Sysclk_source::pll:
            return "PLL";

        default:
            return "UNKNOWN";
    }

    return "";
}

int32_t compute_temperature(const ADC::Calibration_data& a_calibration_data, uint32_t measure)
{
    int32_t temperature = 100 * ((measure * 330 / 300) - a_calibration_data.temperature_sensor_data_1);
    temperature /= (a_calibration_data.temperature_sensor_data_2 - a_calibration_data.temperature_sensor_data_1);

    return temperature + 30;
}

void system_timer_update(void*)
{
    system_timer::update();
}

} // namespace

int main()
{
    using namespace cml;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::debug;

    mcu::register_pre_sysclk_frequency_change_callback({ pre_sysclk_freq_change, nullptr });
    mcu::register_post_sysclk_frequency_change_callback({ post_sysclk_freq_change, nullptr });

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi,
                    { mcu::Bus_prescalers::AHB::_1, mcu::Bus_prescalers::APB1::_1, mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 10u << 4u });
        mcu::disable_msi_clock();

        assert::register_halt({ assert_mcu_halt, nullptr });

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz_to_Hz(1)) - 1, systick::Prescaler::_1, 0x9u);
        systick::register_tick_callback({ system_timer_update, nullptr });

        mcu::enable_dwt(); // uS delay for adc

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        GPIO::Alternate_function::Config usart_pin_config = {
            GPIO::Mode::push_pull, GPIO::Pull::up, GPIO::Speed::high, 0x7u
        };

        gpio_port_a.p_alternate_function->enable(2u, usart_pin_config);
        gpio_port_a.p_alternate_function->enable(3u, usart_pin_config);

        USART iostream(USART::Id::_2);
        bool iostream_ready = iostream.enable({ 115200,
                                                USART::Oversampling::_16,
                                                USART::Stop_bits::_1,
                                                USART::Flow_control_flag::none,
                                                USART::Sampling_method::three_sample_bit,
                                                USART::Mode_flag::tx },
                                              { USART::Word_length::_8_bit, USART::Parity::none },
                                              { USART::Clock::Source::sysclk, mcu::get_sysclk_frequency_hz() },
                                              0x1u,
                                              10u);

        if (true == iostream_ready)
        {
            Logger logger({ write_string, &iostream }, true, true, true, true, true);
            assert::register_print({ assert_print, &iostream });
            logger.inf("CML. CPU speed: %u MHz, source: %s\n",
                       Hz_to_MHz(mcu::get_sysclk_frequency_hz()),
                       sysclk_source_to_cstring(mcu::get_sysclk_source()));

            ADC adc(ADC::Id::_1);
            const ADC::Channel enabled_channels[] = { ADC::Channel::Id::temperature_sensor,
                                                      ADC::Channel::Sampling_time::_640_5_clock_cycles };
            bool periph_ready =
                adc.enable(ADC::Resolution::_12_bit,
                           { ADC::Synchronous_clock::Source::pclk, ADC::Synchronous_clock::Divider::_1 },
                           20,
                           1);

            if (true == periph_ready)
            {
                adc.set_active_channels(enabled_channels, 1);

                while (true)
                {
                    uint16_t r = 0;
                    adc.read_polling(&r, 1);

                    logger.inf("temp: %d, adc: %u\r", compute_temperature(adc.get_calibration_data(), r), r);
                    delay::ms(1000);
                }
            }
            else
            {
                logger.err("Cannot intialize ADC\n");
            }

            while (true)
                ;
        }
    }

    while (true)
        ;
}