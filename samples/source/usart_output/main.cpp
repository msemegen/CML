/*
 *   Name: main.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/debug/assertion.hpp>
#include <cml/hal/Factory.hpp>
#include <cml/hal/GPIO.hpp>
#include <cml/hal/Polling.hpp>
#include <cml/hal/Systick.hpp>
#include <cml/hal/USART.hpp>
#include <cml/hal/internal_flash.hpp>
#include <cml/hal/mcu.hpp>
#include <cml/hal/pwr.hpp>
#include <cml/hal/rcc.hpp>
#include <cml/hal/system_timer.hpp>
#include <cml/utils/delay.hpp>

// debug
#include <cml/hal/DMA.hpp>
#include <cml/hal/I2C.hpp>
#include <cml/hal/Interrupt.hpp>
#include <cml/hal/SPI.hpp>
#include <cml/hal/CRC32.hpp>

namespace {
using namespace cml::hal;

void assert_halt(void*)
{
    mcu::halt();
    while (true)
        ;
}

void assert_print(const char*, uint32_t, const char*, void*) {}

//void usart_callback(std::uint32_t a_data, bool a_idle, void* a_p_user_data)
//{
//    GPIO::Out::Pin* p_led_pin = reinterpret_cast<GPIO::Out::Pin*>(a_p_user_data);
//
//    p_led_pin->toggle_level();
//}
} // namespace

int main()
{
    using namespace cml::hal;
    using namespace cml::debug;
    using namespace cml::utils;

    GPIO::Alternate_function::Enable_config usart_pin_config = {
        GPIO::Mode::push_pull, GPIO::Pull::up, GPIO::Speed::high, 0x7u
    };

    internal_flash::set_latency(internal_flash::Latency::_4);
    pwr::set_voltage_scaling(pwr::Voltage_scaling::_1);

    rcc<mcu>::enable_clock<rcc<mcu>::Clock::HSI>(rcc<mcu>::HSI_frequency::_16_MHz);
    rcc<mcu>::enable_clock<rcc<mcu>::Clock::PLL>(
        rcc<mcu>::PLL_source::HSI,
        rcc<mcu>::PLLM::_2,
        { 20u,
          { rcc<mcu>::PLL_config::R::Divider::_2, rcc<mcu>::PLL_config::Output::enabled },
          { rcc<mcu>::PLL_config::Q::Divider::_2, rcc<mcu>::PLL_config::Output::disabled },
          { rcc<mcu>::PLL_config::P::Divider::_7, rcc<mcu>::PLL_config::Output::disabled } });

    rcc<mcu>::set_SYSCLK_source(
        rcc<mcu>::SYSCLK_source::PLL,
        { rcc<mcu>::Bus_prescalers::AHB::_1, rcc<mcu>::Bus_prescalers::APB1::_1, rcc<mcu>::Bus_prescalers::APB2::_1 });

    rcc<mcu>::disable_clock(rcc<mcu>::Clock::MSI);

    Systick systick;
    systick.enable((rcc<mcu>::get_SYSCLK_frequency_Hz() / 1000u) - 1, Systick::Prescaler::_1, 0x9u);
    systick.register_tick_callback({ system_timer::update, nullptr });

    assertion::register_halt({ assert_halt, nullptr });
    assertion::register_print({ assert_print, nullptr });

    GPIO gpio_port_a = Factory<GPIO, 1>::create();
    rcc<GPIO, 1>::enable(false);
    gpio_port_a.enable();

    gpio_port_a.p_alternate_function->enable(2u, usart_pin_config);
    gpio_port_a.p_alternate_function->enable(3u, usart_pin_config);

    USART usart = Factory<USART, 2>::create();
    rcc<USART, 2>::enable<rcc<USART, 2>::Clock_source::SYSCLK>(false);

    bool usart_ready = usart.enable({ 115200u,
                                      rcc<mcu>::get_SYSCLK_frequency_Hz(),
                                      USART::Enable_config::Oversampling::_16,
                                      USART::Enable_config::Stop_bits::_1,
                                      USART::Enable_config::Flow_control_flag::none,
                                      USART::Enable_config::Sampling_method::three_sample_bit,
                                      USART::Enable_config::Mode_flag::tx | USART::Enable_config::Mode_flag::rx },
                                    { USART::Frame_format::Word_length::_8_bit, USART::Frame_format::Parity::none },
                                    10u);

    if (true == usart_ready)
    {
        GPIO::Out::Pin led_pin;
        gpio_port_a.p_out->enable(5u, { GPIO::Mode::push_pull, GPIO::Pull::down, GPIO::Speed::low }, &led_pin);

        Polling<USART> iostream = Factory<Polling<USART>, 1>::create(&usart);
        Interrupt<USART> it     = Factory<Interrupt<USART>, 1>::create(&usart);

        // it.enable({ true, 0x1u, 0x1u });
        // it.rx.register_callback({ usart_callback, &led_pin });

        Interrupt<GPIO> gpio_itr = Factory<Interrupt<GPIO>>::create<Interrupt<GPIO>::Id::_0>();

        rcc<SPI, 1u>::enable(false);

        SPI_master spi               = Factory<SPI_master, 1>::create();
        Interrupt<SPI_master> spi_it = Factory<Interrupt<SPI_master>, 1>::create(&spi);
        spi_it.enable({});

        spi_it.transmission.tx.register_callback(Interrupt<SPI_master>::Transmission::TX::Callback {});

        I2C_master i2c_master       = Factory<I2C_master, 1>::create();
        Interrupt<I2C_master> i2cit = Factory<Interrupt<I2C_master>, 1>::create(&i2c_master);

        I2C_slave i2c_slave         = Factory<I2C_slave, 2>::create();
        Interrupt<I2C_slave> i2cit2 = Factory<Interrupt<I2C_slave>, 2>::create(&i2c_slave);

        //i2cit.transmission.enable({});

        spi_it.enable({});
        spi_it.transmission.rx.register_callback({});
        spi_it.transmission.tx.register_callback({});

        i2cit.transmission.enable({});
        i2cit.transmission.tx.register_callback(0, 0, {});

        i2cit.enable();

        i2cit2.transmission.enable({});
        i2cit2.transmission.rx.register_callback({});
        i2cit2.transmission.tx.register_callback({});

        i2cit2.transmission.tx.unregister_callback();
        i2cit2.transmission.rx.unregister_callback();

        i2cit2.status.enable({});
        i2cit2.status.disable();

        rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::HSI>(false);
        rcc<SPI, 1>::enable(false);

        spi_it.enable({});
        spi_it.status.register_callback({});

        rcc<CRC32>::enable(false);

        CRC32 crc32 = Factory<CRC32>::create();
       

        //soc::m4::stm32l4::Interrupt<soc::m4::stm32l4::GPIO> gpioit =
        //    soc::m4::stm32l4::Factory<soc::m4::stm32l4::Interrupt<soc::m4::stm32l4::GPIO>>::create<
        //        soc::m4::stm32l4::Interrupt<soc::m4::stm32l4::GPIO>::Id::_1>();

        while (true)
        {
            iostream.transmit("Hello world\n", 12u);
            led_pin.toggle_level();
            delay::ms(500);
        }

        // Interrupt<USART> it(&usart);
        //
        // it.enable({ true, 0x1, 0x1 });
        //
        // it.tx.register_callback({});
        // it.tx.unregister_callback();
        //
        // it.rx.register_callback({});
        // it.rx.unregister_callback();
        //
        // DMA1<USART> dma(&usart, USART::id::_2);
        // dma.rx.enable({});
        //
        // I2C_master i2c_master(I2C_master::id::_1);
        //
        // Interrupt<I2C_master> i2c_it(&i2c_master, I2C_master::id::_1);
        // i2c_it.transmission.enable({});
        // i2c_it.transmission.tx.register_callback({});
    }

    while (true)
        ;

    return 0;
}