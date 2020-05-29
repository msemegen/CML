/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/bit.hpp>
#include <common/frequency.hpp>
#include <hal/GPIO.hpp>
#include <hal/mcu.hpp>
#include <hal/systick.hpp>
#include <hal/USART.hpp>
#include <utils/delay.hpp>
#include <utils/Console.hpp>

#include <hal/I2C.hpp>

//#define MASTER
//#define SLAVE

namespace
{
    using namespace cml::hal::stm32l452xx;
    using namespace cml::utils;

void print_status(Console* a_p_console, const char* a_p_tag, I2C_base::Bus_status a_bus_status, uint32_t a_bytes)
{
    a_p_console->write("[%s] status: ", a_p_tag);

    switch (a_bus_status)
    {
        case I2C_base::Bus_status::ok:
        {
            a_p_console->write("ok ");
        }
        break;

        case I2C_base::Bus_status::timeout:
        {
            a_p_console->write("timeout ");
        }
        break;

        case I2C_base::Bus_status::overrun:
        {
            a_p_console->write("overrun ");
        }
        break;

        case I2C_base::Bus_status::underrun:
        {
            a_p_console->write("underrun ");
        }
        break;

        case I2C_base::Bus_status::arbitration_lost:
        {
            a_p_console->write("arbitration lost ");
        }
        break;

        case I2C_base::Bus_status::misplaced:
        {
            a_p_console->write("misplaced ");
        }
        break;

        case I2C_base::Bus_status::crc_error:
        {
            a_p_console->write("crc error ");
        }
        break;

        case I2C_base::Bus_status::nack:
        {
            a_p_console->write("nack ");
        }
        break;
    }

    a_p_console->write_line("-> bytes: %u", a_bytes);
}

} // namespace ::

int main()
{
    using namespace cml::common;
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

        systick::enable(9u);

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
                                                  USART::Word_length::_8_bits,
                                                  USART::Stop_bits::_1,
                                                  USART::Flow_control::none,
                                                  USART::Parity::none
                                                },
                                                { USART::Clock::Source::sysclk,
                                                  mcu::get_sysclk_frequency_hz(),
                                                },
                                                0x1u, 10);

        if (true == usart_ready)
        {
#ifdef MASTER

            Console console(&console_usart);
            console.write_line("CML I2C master sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            I2C_master i2c_master_bus(I2C_master::Id::_1);
            i2c_master_bus.enable({ false, true, false, 0x00200205 }, I2C_master::Clock_source::sysclk, 0x1u);

            delay::ms(500);

            const uint8 data_to_send[] = { 0x1u, 0x2u };
            uint8 data_to_receive[]    = { 0xFF, 0xFF };

            I2C_master::Bus_status bus_status;

            if (true == i2c_master_bus.is_slave_connected(0x11, 10))
            {
                while (true)
                {
                    uint32 bytes = i2c_master_bus.transmit_bytes_polling(0x11,
                                                                         data_to_send,
                                                                         sizeof(data_to_send),
                                                                         &bus_status);

                    if (I2C_master::Bus_status::ok == bus_status)
                    {
                        bytes += i2c_master_bus.receive_bytes_polling(0x11,
                                                                      &data_to_receive,
                                                                      sizeof(data_to_receive),
                                                                      &bus_status);
                    }


                    print_status(&console, "capcom", bus_status, bytes);
                    delay::ms(1000);
                }
            }
            console.write_line("No slave detected...");
            while (true);

#endif // MASTER

#ifdef SLAVE

            Console console(&console_usart);
            console.write_line("CML I2C slave sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            I2C_slave i2c_slave_bus(I2C_slave::Id::_1);
            i2c_slave_bus.enable({ false, true, false, 0x00200205, 0x11 }, I2C_slave::Clock_source::sysclk, 0x1u);

            const uint8 data_to_send[] = { 0x1u, 0x2u };
            uint8 data_to_receive[]    = { 0xFF, 0xFF };

            I2C_slave::Bus_status bus_status;

            while (true)
            {
                uint32_t bytes_received = i2c_slave_bus.receive_bytes_polling(&data_to_receive,
                                                                              sizeof(data_to_receive),
                                                                              &bus_status);
                print_status(&console, "receive_bytes_polling", bus_status, bytes_received);

                uint32 bytes_transmited = i2c_slave_bus.transmit_bytes_polling(&data_to_send,
                                                                               sizeof(data_to_send),
                                                                               &bus_status);
                print_status(&console, "transmit_bytes_polling", bus_status, bytes_transmited);
            }


#endif // SLAVE
        }
    }

    while (true);
}