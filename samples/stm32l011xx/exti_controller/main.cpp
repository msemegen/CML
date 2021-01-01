// cml
#include <cml/debug/assert.hpp>
#include <cml/frequency.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/hal/systick.hpp>
#include <cml/utils/Logger.hpp>
#include <cml/various.hpp>

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

void system_timer_update(void*)
{
    system_timer::update();
}

void exti_callback(GPIO::Level, void* a_p_user_data)
{
    reinterpret_cast<GPIO::Out::Pin*>(a_p_user_data)->toggle_level();
}

} // namespace

int main()
{
    using namespace cml;
    using namespace cml::debug;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;

    mcu::register_pre_sysclk_frequency_change_callback({ pre_sysclk_freq_change, nullptr });
    mcu::register_post_sysclk_frequency_change_callback({ post_sysclk_freq_change, nullptr });

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi,
                    { mcu::Bus_prescalers::AHB::_1, mcu::Bus_prescalers::APB1::_1, mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::disable_msi_clock();

        assert::register_halt({ assert_mcu_halt, nullptr });

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz_to_Hz(1)) - 1, systick::Prescaler::_1, 0x9u);
        systick::register_tick_callback({ system_timer_update, nullptr });

        mcu::enable_syscfg();
        mcu::enable_interrupt_line(mcu::Interrupt_line::exti_4_15, 0x5u);

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

            GPIO gpio_port_b(GPIO::Id::b);
            gpio_port_b.enable();

            GPIO::Out::Pin led_pin;
            gpio_port_b.p_out->enable(3u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);
            led_pin.set_level(GPIO::Level::low);

            GPIO::In::Pin in_pin;
            gpio_port_a.p_in->enable(9u, GPIO::Pull::down, &in_pin);
            in_pin.register_interrupt_callback(GPIO::In::Pin::Interrupt_mode_flag::rising, { exti_callback, &led_pin });

            while (true)
                ;
        }
    }

    while (true)
        ;
}