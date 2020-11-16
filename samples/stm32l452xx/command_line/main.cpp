// cml
#include <cml/cstring.hpp>
#include <cml/debug/assert.hpp>
#include <cml/frequency.hpp>
#include <cml/hal/counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/systick.hpp>
#include <cml/utils/Command_line.hpp>
#include <cml/utils/Logger.hpp>

namespace {

using namespace cml;
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

uint32_t write_character(char a_character, void* a_p_user_data)
{
    return reinterpret_cast<USART*>(a_p_user_data)->transmit_bytes_polling(&a_character, 1).data_length_in_words;
}

uint32_t write_string(const char* a_p_string, uint32_t a_length, void* a_p_user_data)
{
    return reinterpret_cast<USART*>(a_p_user_data)->transmit_bytes_polling(a_p_string, a_length).data_length_in_words;
}

uint32_t read_key(char* a_p_out, uint32_t a_length, void* a_p_user_data)
{
    return reinterpret_cast<USART*>(a_p_user_data)->receive_bytes_polling(a_p_out, a_length).data_length_in_words;
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

void led_cli_callback(const Command_line::Callback::Parameter* a_p_params, uint32_t a_count, void* a_p_user_data)
{
    pin::Out* p_led_pin = reinterpret_cast<pin::Out*>(a_p_user_data);

    if (2 == a_count)
    {
        bool is_on = cstring::equals(a_p_params[1].a_p_value, "on", a_p_params[1].length);

        if (true == is_on)
        {
            p_led_pin->set_level(pin::Level::high);
        }
        else
        {
            bool is_off = cstring::equals(a_p_params[1].a_p_value, "off", a_p_params[1].length);

            if (true == is_off)
            {
                p_led_pin->set_level(pin::Level::low);
            }
        }
    }
}

void reset_callback(const Command_line::Callback::Parameter* a_p_params, uint32_t a_count, void* a_p_user_data)
{
    mcu::reset();
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

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz_to_Hz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        gpio_port_a.enable();

        pin::af::Config usart_pin_config = { pin::Mode::push_pull, pin::Pull::up, pin::Speed::high, 0x7u };
        pin::af::enable(&gpio_port_a, 2, usart_pin_config);
        pin::af::enable(&gpio_port_a, 3, usart_pin_config);

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
            Logger logger({ write_string, &iostream }, true, true, true, true, true);
            assert::register_print({ assert_print, &iostream });
            logger.inf("CML. CPU speed: %u MHz, source: %s\n",
                       Hz_to_MHz(mcu::get_sysclk_frequency_hz()),
                       sysclk_source_to_cstring(mcu::get_sysclk_source()));

            Command_line command_line({ write_character, &iostream },
                                      { write_string, &iostream },
                                      { read_key, &iostream },
                                      "cmd > ",
                                      "Command not found");

            pin::Out led_pin;
            pin::out::enable(&gpio_port_a, 5, { pin::Mode::push_pull, pin::Pull::down, pin::Speed::low }, &led_pin);

            command_line.register_callback({ "led", led_cli_callback, &led_pin });
            command_line.register_callback({ "reset", reset_callback, nullptr });

            command_line.write_prompt();

            while (true)
            {
                command_line.update();
            }
        }
    }

    while (true)
        ;
}