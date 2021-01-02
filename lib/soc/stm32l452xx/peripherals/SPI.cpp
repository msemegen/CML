/*
    Name: SPI.cpp

    Copyright(c) 2021 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <soc/stm32l452xx/peripherals/SPI.hpp>

//soc
#include <soc/system_timer.hpp>

//cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/various.hpp>
#include <cml/utils/wait.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l452xx::peripherals;

struct Controller
{
    using Enable_function  = void (*)(uint32_t a_irq_priority);
    using Disable_function = void (*)();

    SPI_TypeDef* p_registers        = nullptr;
    SPI_master* p_spi_master_handle = nullptr;
    SPI_slave* p_spi_slave_handle   = nullptr;

    Enable_function enable   = nullptr;
    Disable_function disable = nullptr;
};

void spi_1_enable(uint32_t a_irq_priority)
{
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);

    NVIC_SetPriority(SPI1_IRQn, a_irq_priority);
    NVIC_EnableIRQ(SPI1_IRQn);
}

void spi_1_disable()
{
    bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);
    NVIC_DisableIRQ(SPI1_IRQn);
}

void spi_2_enable(uint32_t a_irq_priority)
{
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN);

    NVIC_SetPriority(SPI2_IRQn, a_irq_priority);
    NVIC_EnableIRQ(SPI2_IRQn);
}

void spi_2_disable()
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN);
    NVIC_DisableIRQ(SPI2_IRQn);
}

void spi_3_enable(uint32_t a_irq_priority)
{
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI3EN);

    NVIC_SetPriority(SPI3_IRQn, a_irq_priority);
    NVIC_EnableIRQ(SPI3_IRQn);
}

void spi_3_disable()
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI3EN);
    NVIC_DisableIRQ(SPI3_IRQn);
}

Controller controllers[] { { SPI1, nullptr, nullptr, spi_1_enable, spi_1_disable },
                           { SPI2, nullptr, nullptr, spi_2_enable, spi_2_disable },
                           { SPI3, nullptr, nullptr, spi_3_enable, spi_3_disable } };

SPI_TypeDef* get_spi_ptr(SPI_base::Id a_id)
{
    return controllers[static_cast<uint32_t>(a_id)].p_registers;
}

bool is_SPI_SR_error(SPI_base::Id a_id)
{
    return bit::is_any(get_spi_ptr(a_id)->SR, SPI_SR_FRE | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR);
}

SPI_base::Result::Bus_flag get_bus_flag_from_SPI_SR(SPI_base::Id a_id) 
{
    SPI_base::Result::Bus_flag ret = SPI_base::Result::Bus_flag::ok;
    uint32_t sr                    = get_spi_ptr(a_id)->SR;

    if (true == bit_flag::is(sr, SPI_SR_OVR))
    {
        ret |= SPI_base::Result::Bus_flag::overrun;
    }

    if (true == bit_flag::is(sr, SPI_SR_FRE))
    {
        ret |= SPI_base::Result::Bus_flag::frame_error;
    }

    if (true == bit_flag::is(sr, SPI_SR_MODF))
    {
        ret |= SPI_base::Result::Bus_flag::mode_fault;
    }

    if (true == bit_flag::is(sr, SPI_SR_CRCERR))
    {
        ret |= SPI_base::Result::Bus_flag::crc_error;
    }

    return ret;
}

void clear_SPI_SR_errors(SPI_base::Id a_id)
{
    uint32_t sr = get_spi_ptr(a_id)->SR;

    if (true == bit_flag::is(sr, SPI_SR_OVR))
    {
        volatile uint32_t t1 = get_spi_ptr(a_id)->DR;
        volatile uint32_t t2 = get_spi_ptr(a_id)->SR;

        unused(t1);
        unused(t2);
    }

    if (true == bit_flag::is(sr, SPI_SR_MODF))
    {
        volatile uint32_t t1 = get_spi_ptr(a_id)->SR;
        unused(t1);

        bit_flag::set(&(get_spi_ptr(a_id)->CR1), SPI_CR1_SPE);
    }

    if (true == bit_flag::is(sr, SPI_SR_FRE))
    {
        volatile uint32_t t1 = get_spi_ptr(a_id)->SR;
        unused(t1);
    }

    if (true == bit_flag::is(sr, SPI_SR_CRCERR))
    {
        bit::clear(&(get_spi_ptr(a_id)->SR), SPI_SR_CRCERR_Pos);
    }
}

} // namespace

namespace soc {
namespace stm32l452xx {
namespace peripherals {

using namespace cml;
using namespace cml::utils;

void SPI_master::enable(const Config& a_config,
                        const Frame_format& a_frame_format,
                        const Clock_source& a_clock_source,
                        uint32_t a_irq_priority)
{
    controllers[static_cast<uint32_t>(this->id)].enable(a_irq_priority);
    controllers[static_cast<uint32_t>(this->id)].p_spi_slave_handle  = nullptr;
    controllers[static_cast<uint32_t>(this->id)].p_spi_master_handle = this;

    get_spi_ptr(this->id)->CR1 =
        static_cast<uint32_t>(a_config.mode) | static_cast<uint32_t>(a_config.wiring) |
        static_cast<uint32_t>(a_frame_format.bit_significance) |
        static_cast<uint32_t>(a_clock_source.prescaler) | (true == a_config.crc_enable ? SPI_CR1_CRCEN : 0x0u) |
        SPI_CR1_SSM | SPI_CR1_MSTR;

    get_spi_ptr(this->id)->CR2 = static_cast<uint32_t>(a_frame_format.word_length) | SPI_CR2_SSOE |
                                 (a_frame_format.word_length <= Frame_format::Word_length::_8 ? SPI_CR2_FRXTH : 0x0u);
    get_spi_ptr(this->id)->CR1 |= SPI_CR1_SPE;
}

void SPI_master::disable()
{
    /*
    * The correct disable procedure is (except when receive only mode is used):
    * 1. Wait until FTLVL[1:0] = 00 (no more data to transmit).
    * 2. Wait until BSY=0 (the last data frame is processed).
    * 3. Disable the SPI (SPE=0).
    * 4. Read data until FRLVL[1:0] = 00 (read all the received data).
    * The correct disable procedure for certain receive only modes is:
    * 1. Interrupt the receive flow by disabling SPI (SPE=0) in the specific time window while
    * the last data frame is ongoing.
    * 2. Wait until BSY=0 (the last data frame is processed).
    * 3. Read data until FRLVL[1:0] = 00 (read all the received data).
    */

    wait::until(&(get_spi_ptr(this->id)->SR), SPI_SR_FRLVL, false);

    get_spi_ptr(this->id)->CR2 = 0;
    get_spi_ptr(this->id)->CR1 = 0;

    controllers[static_cast<uint32_t>(this->id)].disable();
    controllers[static_cast<uint32_t>(this->id)].p_spi_master_handle = nullptr;
}

SPI_master::Result
SPI_master::transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);

    if (nullptr != a_p_nss)
    {
        a_p_nss->set_level(GPIO::Level::low);
    }

    bool error                = false;
    uint32_t words            = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    while (true == bit::is_any(get_spi_ptr(this->id)->SR, SPI_SR_FRLVL) && false == error)
    {
        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (this->frame_format.word_length <= Frame_format::Word_length::_8)
            {
                get_spi_ptr(this->id)->DR = static_cast<const uint8_t*>(a_p_data)[words++];
            }
            else
            {
                get_spi_ptr(this->id)->DR = static_cast<const uint16_t*>(a_p_data)[words++];
            }
        }

        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            volatile uint32_t tmp = get_spi_ptr(this->id)->DR;
            unused(tmp);
        }

        error = is_SPI_SR_error(this->id);
    }

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
        }
    }

    if (nullptr != a_p_nss)
    {
        a_p_nss->set_level(GPIO::Level::high);
    }

    return { bus_flag, words };
}

SPI_master::Result SPI_master::transmit_bytes_polling(const void* a_p_data,
                                                      uint32_t a_data_size_in_words,
                                                      time::tick a_timeout,
                                                      GPIO::Out::Pin* a_p_nss)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);

    time::tick start = system_timer::get();

    if (nullptr != a_p_nss)
    {
        a_p_nss->set_level(GPIO::Level::low);
    }

    bool error                = false;
    uint32_t words            = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    while (true == bit::is_any(get_spi_ptr(this->id)->SR, SPI_SR_FRLVL) && false == error &&
           a_timeout < time::diff(system_timer::get(), start))
    {
        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE) && words < a_data_size_in_words)
        {
            if (this->frame_format.word_length <= Frame_format::Word_length::_8)
            {
                get_spi_ptr(this->id)->DR = static_cast<const uint8_t*>(a_p_data)[words++];
            }
            else
            {
                get_spi_ptr(this->id)->DR = static_cast<const uint16_t*>(a_p_data)[words++];
            }
        }

        if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
        {
            volatile uint32_t tmp = get_spi_ptr(this->id)->DR;
            unused(tmp);
        }

        error = is_SPI_SR_error(this->id);
    }

    if (true == error)
    {
        bus_flag = get_bus_flag_from_SPI_SR(this->id);

        if (Result::Bus_flag::ok != bus_flag)
        {
            clear_SPI_SR_errors(this->id);
        }
    }

    if (nullptr != a_p_nss)
    {
        a_p_nss->set_level(GPIO::Level::high);
    }

    return { bus_flag, words };
}

SPI_master::Result
SPI_master::receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss)
{
    assert(nullptr != a_p_data);
    assert(a_data_size_in_words > 0);

    if (nullptr != a_p_nss)
    {
        a_p_nss->set_level(GPIO::Level::low);
    }

    bool error                = false;
    bool transmit_enable      = true;
    uint32_t words            = 0;
    Result::Bus_flag bus_flag = Result::Bus_flag::ok;

    if (Config::Wiring::full_duplex == this->confg.wiring)
    {
        while (false == error && words < a_data_size_in_words)
        {
            if (true == transmit_enable && true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_TXE))
            {
                if (a_data_size_in_words - words > 1)
                {
                    *(reinterpret_cast<volatile uint16_t*>(&(get_spi_ptr(this->id)->DR))) =
                        static_cast<uint16_t>(0xFFFFu);
                }
                else
                {
                   *(reinterpret_cast<volatile uint8_t*>(&(get_spi_ptr(this->id)->DR))) = static_cast<uint8_t>(0xFFu);
                }

                transmit_enable = false;
            }

            if (true == bit_flag::is(get_spi_ptr(this->id)->SR, SPI_SR_RXNE))
            {
                if (a_data_size_in_words - words > 1)
                {
                    static_cast<uint16_t*>(a_p_data)[words] = static_cast<uint16_t>(get_spi_ptr(this->id)->DR);
                    words += 2;
                }
                else
                {
                    static_cast<uint8_t*>(a_p_data)[words++] = static_cast<uint8_t>(get_spi_ptr(this->id)->DR);
                }

                transmit_enable = true;
            }
        }
    }
    else
    {
    }

    words++;

    if (nullptr != a_p_nss)
    {
        a_p_nss->set_level(GPIO::Level::high);
    }

    return { bus_flag, error };
}

SPI_master::Result SPI_master::receive_bytes_polling(void* a_p_data,
                                                     uint32_t a_data_size_in_words,
                                                     time::tick a_timeout,
                                                     GPIO::Out::Pin* a_p_nss)
{
    if (nullptr != a_p_nss)
    {

    }

    if (nullptr != a_p_nss)
    {
    }

    return Result();
}

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx