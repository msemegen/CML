/*
    Name: main.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/bit.hpp>
#include <cml/frequency.hpp>
#include <cml/hal/counter.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/systick.hpp>
#include <cml/hal/peripherals/GPIO.hpp>
#include <cml/hal/peripherals/I2C.hpp>
#include <cml/hal/peripherals/USART.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/Console.hpp>

//#define MASTER
#define SLAVE

namespace
{

using namespace cml;
using namespace cml::hal::peripherals;
using namespace cml::utils;

void print_status(Console* a_p_console, const char* a_p_tag, I2C_base::Bus_status_flag a_bus_status, uint32_t a_bytes)
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

class Tx_context
{
public:

    void inc()
    {
        this->data[0]++;
        this->data[1]++;
    }

    void set_stop()
    {
        this->stop = true;
    }

    uint8_t get_back() const
    {
        assert(this->i < sizeof(this->data));

        return this->data[this->i++];
    }

    void reset()
    {
        this->i = 0;
        this->stop = false;
    }

    bool is_empty() const
    {
        return this->i == sizeof(this->data);
    }

    uint32_t get_i() const
    {
        return this->i;
    }

    constexpr uint32_t get_length() const
    {
        return sizeof(this->data);
    }

    bool is_stop() const
    {
        return this->stop;
    }

private:

    volatile mutable uint32_t i = 0;
    volatile bool stop = false;
    uint8_t data[2] = { 0x0u, 0x0u };
};

class Rx_context
{
public:

    void set(uint8_t a_value)
    {
        assert(this->i < sizeof(this->data));

        data[this->i++] = a_value;
    }

    void set_stop()
    {
        this->stop = true;
    }

    uint8_t get(uint32_t a_idx) const
    {
        assert(a_idx < sizeof(this->data));

        return this->data[a_idx];
    }

    void reset()
    {
        this->i = 0;
        this->stop = false;
    }

    bool is_full() const
    {
        return this->i == sizeof(this->data);
    }

    uint32_t get_i() const
    {
        return this->i;
    }

    constexpr uint32_t get_length() const
    {
        return sizeof(this->data);
    }

    bool is_stop() const
    {
        return this->stop;
    }

private:

    volatile uint32_t i = 0;
    volatile bool stop = false;
    uint8_t data[2] = { 0x0u, 0x0u };
};

volatile I2C_master::Bus_status_flag bus_status = I2C_master::Bus_status_flag::ok;

void rx_callback(uint8_t a_value, bool a_stop, void* a_p_user_data)
{
    Rx_context* p_rx_context = reinterpret_cast<Rx_context*>(a_p_user_data);

    if (false == a_stop && false == p_rx_context->is_full())
    {
        p_rx_context->set(a_value);
    }
    else if(true == a_stop)
    {
        p_rx_context->set_stop();
    }
}

void tx_callback(volatile uint32_t* a_p_data, bool a_stop, void* a_p_user_data)
{
    Tx_context* p_tx_context = reinterpret_cast<Tx_context*>(a_p_user_data);

    if (false == a_stop && false == p_tx_context->is_empty())
    {
        (*a_p_data) = p_tx_context->get_back();
    }
    else if (true == a_stop)
    {
        p_tx_context->set_stop();
    }
}

bool bus_status_callback(I2C_base::Bus_status_flag a_bus_status, void*)
{
    bus_status = a_bus_status;
    return true;
}

uint32_t write_character(char a_character, void* a_p_user_data)
{
    USART* p_console_usart = reinterpret_cast<USART*>(a_p_user_data);
    return p_console_usart->transmit_bytes_polling(&a_character, 1).data_length_in_words;
}

uint32_t write_string(const char* a_p_string, uint32_t a_length, void* a_p_user_data)
{
    USART* p_console_usart = reinterpret_cast<USART*>(a_p_user_data);
    return p_console_usart->transmit_bytes_polling(a_p_string, a_length).data_length_in_words;
}

uint32_t read_key(char* a_p_out, uint32_t a_length, void* a_p_user_data)
{
    USART* p_console_usart = reinterpret_cast<USART*>(a_p_user_data);
    return p_console_usart->receive_bytes_polling(a_p_out, a_length).data_length_in_words;
}

} // namespace ::

int main()
{
    using namespace cml;
    using namespace cml::hal;
    using namespace cml::hal::peripherals;
    using namespace cml::utils;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::disable_msi_clock();
        mcu::enable_syscfg();

        systick::enable((mcu::get_sysclk_frequency_hz() / kHz(1)) - 1, 0x9u);
        systick::register_tick_callback({ counter::update, nullptr });

        GPIO gpio_port_a(GPIO::Id::a);
        GPIO gpio_port_b(GPIO::Id::b);

        gpio_port_a.enable();
        gpio_port_b.enable();

        Alternate_function_pin console_usart_tx_pin(&gpio_port_a, 2);
        Alternate_function_pin console_usart_rx_pin(&gpio_port_a, 15);

        Alternate_function_pin i2c_scl_pin(&gpio_port_a, 9);
        Alternate_function_pin i2c_sda_pin(&gpio_port_a, 10);

        console_usart_tx_pin.enable({ Alternate_function_pin::Mode::push_pull,
                                      Alternate_function_pin::Pull::up,
                                      Alternate_function_pin::Speed::low,
                                      0x4u
                                   });

        console_usart_rx_pin.enable({ Alternate_function_pin::Mode::push_pull,
                                      Alternate_function_pin::Pull::up,
                                      Alternate_function_pin::Speed::low,
                                      0x4u
                                   });

        i2c_scl_pin.enable({ Alternate_function_pin::Mode::open_drain,
                             Alternate_function_pin::Pull::up,
                             Alternate_function_pin::Speed::high,
                             0x1u
                          });

        i2c_sda_pin.enable({ Alternate_function_pin::Mode::open_drain,
                             Alternate_function_pin::Pull::up,
                             Alternate_function_pin::Speed::high,
                             0x1u
                          });

        USART console_usart(USART::Id::_2);
        bool usart_ready = console_usart.enable({ 115200u,
                                                  USART::Oversampling::_16,
                                                  USART::Stop_bits::_1,
                                                  USART::Flow_control_flag::none,
                                                  USART::Sampling_method::three_sample_bit,
                                                  USART::Mode_flag::tx
                                                },

                                                { USART::Word_length::_8_bit,
                                                  USART::Parity::none
                                                },

                                                { USART::Clock::Source::sysclk,
                                                  mcu::get_sysclk_frequency_hz(),
                                                },
                                                0x1u, 10);

        if (true == usart_ready)
        {
#if defined MASTER && !defined SLAVE

            Console console({ write_character, &console_usart },
                            { write_string,    &console_usart },
                            { read_key,        &console_usart });
            console.write_line("CML I2C master sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            I2C_master i2c_master_bus(I2C_master::Id::_1);
            i2c_master_bus.enable({ false, true, false, 0x00200205 }, I2C_master::Clock_source::sysclk, 0x1u);

            delay::ms(500);

            Rx_context rx_context;
            Tx_context tx_context;

            if (true == i2c_master_bus.is_slave_connected(0x11, 10))
            {
                console.write_line("Slave detected!");

                i2c_master_bus.register_bus_status_callback({ bus_status_callback, nullptr });

                while (true)
                {
                    i2c_master_bus.register_transmit_callback(0x11,
                                                              { tx_callback, &(tx_context) },
                                                              tx_context.get_length());

                    while (false == tx_context.is_stop() && I2C_master::Bus_status_flag::ok == bus_status);
                    uint32_t r = tx_context.get_i();
                    tx_context.reset();

                    if (I2C_master::Bus_status_flag::ok == bus_status)
                    {
                        i2c_master_bus.register_receive_callback(0x11,
                                                                 { rx_callback, &(rx_context) },
                                                                 rx_context.get_length());

                        while (false == rx_context.is_stop() && I2C_master::Bus_status_flag::ok == bus_status);
                        r += rx_context.get_i();
                        rx_context.reset();

                        console.write_line("resp: %u %u", rx_context.get(0), rx_context.get(1));
                    }

                    print_status(&console, "capcom", bus_status, r);
                    bus_status = I2C_master::Bus_status_flag::ok;
                    delay::ms(1000);
                }
            }
            console.write_line("No slave detected...");
            while (true);

#endif // defined MASTER && !defined SLAVE

#if defined SLAVE && !defined MASTER

            Console console({ write_character, &console_usart },
                            { write_string,    &console_usart },
                            { read_key,        &console_usart });
            console.write_line("CML I2C slave sample. CPU speed: %u MHz", mcu::get_sysclk_frequency_hz() / MHz(1));

            I2C_slave i2c_slave_bus(I2C_slave::Id::_1);
            i2c_slave_bus.enable({ false, true, false, 0x00200205, 0x11 }, I2C_slave::Clock_source::sysclk, 0x1u);

            i2c_slave_bus.register_bus_status_callback({ bus_status_callback, nullptr });

            Rx_context rx_context;
            Tx_context tx_context;

            while (true)
            {
                i2c_slave_bus.register_receive_callback({ rx_callback, &(rx_context) }, rx_context.get_length());
                while (false == rx_context.is_stop() && I2C_slave::Bus_status_flag::ok == bus_status);
                uint32_t r = rx_context.get_i();

                if (true == rx_context.is_full() && I2C_slave::Bus_status_flag::ok == bus_status)
                {
                    rx_context.reset();
                    print_status(&console, "register_receive_callback", bus_status, r);

                    i2c_slave_bus.register_transmit_callback( { tx_callback, &(tx_context) }, tx_context.get_length());
                    while (false == tx_context.is_stop() && I2C_master::Bus_status_flag::ok == bus_status);
                    r = tx_context.get_i();
                    tx_context.inc();

                    print_status(&console, "transmit_bytes_polling", bus_status, r);

                    tx_context.reset();
                }
                else
                {
                    if (false == rx_context.is_full() && I2C_slave::Bus_status_flag::ok == bus_status)
                    {
                        console.write_line("Discovered!");
                    }
                    else
                    {
                        print_status(&console, "receive_bytes_polling", bus_status, r);
                    }

                    rx_context.reset();
                }

                bus_status = I2C_slave::Bus_status_flag::ok;
            }

#endif // defined SLAVE && !defined MASTER
        }
    }

    while (true);
}