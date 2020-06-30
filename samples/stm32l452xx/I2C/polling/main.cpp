/*
    Name: main.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/bit.hpp>
#include <cml/frequency.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/I2C.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/counter.hpp>
#include <cml/hal/systick.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/Buffered_console.hpp>

#define MASTER
//#define SLAVE

namespace
{
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

void print_status(Buffered_console* a_p_console,
                  const char* a_p_tag,
                  I2C_base::Bus_status_flag a_bus_status,
                  uint32_t a_bytes)
{
    a_p_console->write("[%s] status: ", a_p_tag);

    switch (a_bus_status)
    {
        case I2C_base::Bus_status_flag::ok:
        {
            a_p_console->write("ok ");
        }
        break;

        case I2C_base::Bus_status_flag::buffer_error:
        {
            a_p_console->write("overrun / underrun ");
        }
        break;

        case I2C_base::Bus_status_flag::arbitration_lost:
        {
            a_p_console->write("arbitration lost ");
        }
        break;

        case I2C_base::Bus_status_flag::misplaced:
        {
            a_p_console->write("misplaced ");
        }
        break;

        case I2C_base::Bus_status_flag::crc_error:
        {
            a_p_console->write("crc error ");
        }
        break;

        case I2C_base::Bus_status_flag::nack:
        {
            a_p_console->write("nack ");
        }
        break;

        case I2C_base::Bus_status_flag::unknown:
        {
            a_p_console->write("unknown ");
        }
        break;
    }

    a_p_console->write_line("-> bytes: %u", a_bytes);
}

} // namespace ::

int main()
{
    using namespace cml;
    using namespace cml::hal;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 10u << 4u });

        mcu::disable_msi_clock();
        mcu::enable_syscfg();

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        GPIO gpio_port_b(GPIO::Id::b);

        gpio_port_a.enable();
        gpio_port_b.enable();

        Alternate_function_pin console_usart_tx_pin(&gpio_port_a, 2);
        Alternate_function_pin console_usart_rx_pin(&gpio_port_a, 3);

        Alternate_function_pin i2c_scl_pin(&gpio_port_b, 8);
        Alternate_function_pin i2c_sda_pin(&gpio_port_b, 9);

        console_usart_tx_pin.enable({ Alternate_function_pin::Mode::push_pull,
                                      Alternate_function_pin::Pull::up,
                                      Alternate_function_pin::Speed::low,
                                      0x7u
                                   });

        console_usart_rx_pin.enable({ Alternate_function_pin::Mode::push_pull,
                                      Alternate_function_pin::Pull::up,
                                      Alternate_function_pin::Speed::low,
                                      0x7u
                                   });

        i2c_scl_pin.enable({ Alternate_function_pin::Mode::open_drain,
                             Alternate_function_pin::Pull::up,
                             Alternate_function_pin::Speed::high,
                             0x4u
                          });

        i2c_sda_pin.enable({ Alternate_function_pin::Mode::open_drain,
                             Alternate_function_pin::Pull::up,
                             Alternate_function_pin::Speed::high,
                             0x4u
                          });

        USART console_usart(USART::Id::_2);
        bool usart_ready = console_usart.enable({ 115200u,
                                                  USART::Oversampling::_16,
                                                  USART::Stop_bits::_1,
                                                  USART::Flow_control_flag::none,
                                                  USART::Parity::none,
                                                  USART::Sampling_method::three_sample_bit
                                                },
                                                { USART::Clock::Source::sysclk,
                                                  mcu::get_sysclk_frequency_hz(),
                                                },
                                                0x1u, 10);

        if (true == usart_ready)
        {
#if defined MASTER && !defined SLAVE

            Buffered_console console(&console_usart);

            console.enable();
            console.write_line("CML I2C master sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            I2C_master i2c_master_bus(I2C_master::Id::_1);
            i2c_master_bus.enable({ false, true, false, 0x00200205 }, I2C_master::Clock_source::sysclk, 0x1u);

            delay::ms(500);

            const uint8 data_to_send[] = { 0x1u, 0x2u };
            uint8 data_to_receive[]    = { 0xFF, 0xFF };

            if (true == i2c_master_bus.is_slave_connected(0x11, 10))
            {
                console.write_line("Slave detected!");

                uint32 bytes = 0;

                while (true)
                {
                    auto result = i2c_master_bus.transmit_bytes_polling(0x11,
                                                                        data_to_send,
                                                                        sizeof(data_to_send),
                                                                        10);

                    if (I2C_master::Bus_status_flag::ok == result.bus_status)
                    {
                        bytes = result.data_length;

                        result = i2c_master_bus.receive_bytes_polling(0x11,
                                                                      data_to_receive,
                                                                      sizeof(data_to_receive),
                                                                      10);

                        if (result.bus_status == I2C_master::Bus_status_flag::ok)
                        {
                            bytes += result.data_length;

                            console.write_line("resp: %u %u", data_to_receive[0], data_to_receive[1]);
                        }
                    }


                    print_status(&console, "capcom", result.bus_status, bytes);
                    delay::ms(1000);
                }
            }
            console.write_line("No slave detected...");
            while (true);

#endif // defined MASTER && !defined SLAVE

#if defined SLAVE && !defined MASTER

            Buffered_console console(&console_usart);

            console.enable();
            console.write_line("CML I2C slave sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            I2C_slave i2c_slave_bus(I2C_slave::Id::_1);
            i2c_slave_bus.enable({ false, true, false, 0x00200205, 0x11 }, I2C_slave::Clock_source::sysclk, 0x1u);

            uint8 data_to_send[]    = { 0x00u, 0x00u };
            uint8 data_to_receive[] = { 0xFFu, 0xFFu };

            while (true)
            {
                auto result = i2c_slave_bus.receive_bytes_polling(&data_to_receive, sizeof(data_to_receive));

                if (result.data_length > 0 && I2C_slave::Bus_status_flag::ok == result.bus_status)
                {

                    print_status(&console, "receive_bytes_polling", result.bus_status, result.data_length);

                    result = i2c_slave_bus.transmit_bytes_polling(&data_to_send, sizeof(data_to_send));

                    data_to_send[0]++;
                    data_to_send[1]++;

                    print_status(&console, "transmit_bytes_polling", result.bus_status, result.data_length);
                }
                else
                {
                    if (0 == result.data_length && I2C_slave::Bus_status_flag::ok == result.bus_status)
                    {
                        console.write_line("Discovered!");
                    }
                    else
                    {
                        print_status(&console, "receive_bytes_polling", result.bus_status, result.data_length);
                    }
                }
            }

#endif // defined SLAVE && !defined MASTER
        }
    }

    while (true);
}