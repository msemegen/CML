#pragma once

/*
 *   Name: Polling.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstddef>

// soc
#include <soc/m4/stm32l4/GPIO/GPIO.hpp>
#include <soc/m4/stm32l4/Polling.hpp>
#include <soc/m4/stm32l4/SPI/SPI.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Polling<SPI_master> : private cml::Non_copyable
{
public:
    struct Result
    {
        enum class Bus_flag : std::uint32_t
        {
            ok          = 0x0u,
            overrun     = 0x1u,
            crc_error   = 0x2u,
            frame_error = 0x4u,
            mode_fault  = 0x8u,
        };

        Bus_flag bus_flag                = cml::various::get_enum_incorrect_value<Bus_flag>();
        std::size_t data_length_in_words = 0;
    };

    Polling(SPI_master* a_p_SPI_master)
        : p_SPI(a_p_SPI_master)
    {
    }

    Result transmit(const void* a_p_data, std::size_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss = nullptr);
    Result transmit(const void* a_p_data,
                    std::size_t a_data_size_in_words,
                    std::uint32_t a_timeout,
                    GPIO::Out::Pin* a_p_nss = nullptr);

    Result receive(void* a_p_data, std::size_t a_data_size_in_words, GPIO::Out::Pin* a_p_nss = nullptr);
    Result receive(void* a_p_data,
                   std::size_t a_data_size_in_words,
                   std::uint32_t a_timeout,
                   GPIO::Out::Pin* a_p_nss = nullptr);

    Result transmit_receive(const void* a_p_tx_data,
                            void* a_p_rx_data,
                            std::size_t a_tx_rx_data_size_in_words,
                            GPIO::Out::Pin* a_p_nss = nullptr);

    Result transmit_receive(const void* a_p_tx_data,
                            void* a_p_rx_data,
                            std::size_t a_tx_rx_data_size_in_words,
                            std::uint32_t a_timeout,
                            GPIO::Out::Pin* a_p_nss = nullptr);

private:
    SPI_master* p_SPI;
};

template<> class Polling<SPI_slave> : private cml::Non_copyable
{
public:
    struct Result
    {
        enum class Bus_flag : std::uint32_t
        {
            ok          = 0x0u,
            overrun     = 0x1u,
            crc_error   = 0x2u,
            frame_error = 0x4u,
            mode_fault  = 0x8u,
        };

        Bus_flag bus_flag                = cml::various::get_enum_incorrect_value<Bus_flag>();
        std::size_t data_length_in_words = 0;
    };

    Polling(SPI_slave* p_SPI_slave)
        : p_SPI(p_SPI_slave)
    {
    }

    Result transmit(const void* a_p_data, std::size_t a_data_size_in_words);
    Result transmit(const void* a_p_data, std::size_t a_data_size_in_words, std::uint32_t a_timeout);

    Result receive(void* a_p_data, std::size_t a_data_size_in_words);
    Result receive(void* a_p_data, std::size_t a_data_size_in_words, std::uint32_t a_timeout);

    Result transmit_receive(const void* a_p_tx_data, void* a_p_rx_data, std::size_t a_tx_rx_data_size_in_words);
    Result transmit_receive(const void* a_p_tx_data,
                            void* a_p_rx_data,
                            std::size_t a_tx_rx_data_size_in_words,
                            std::uint32_t a_timeout);

private:
    SPI_slave* p_SPI;
};

constexpr Polling<SPI_master>::Result::Bus_flag operator|(Polling<SPI_master>::Result::Bus_flag a_f1,
                                                          Polling<SPI_master>::Result::Bus_flag a_f2)
{
    return static_cast<Polling<SPI_master>::Result::Bus_flag>(static_cast<std::uint32_t>(a_f1) |
                                                              static_cast<std::uint32_t>(a_f2));
}
constexpr Polling<SPI_master>::Result::Bus_flag operator&(Polling<SPI_master>::Result::Bus_flag a_f1,
                                                          Polling<SPI_master>::Result::Bus_flag a_f2)
{
    return static_cast<Polling<SPI_master>::Result::Bus_flag>(static_cast<std::uint32_t>(a_f1) &
                                                              static_cast<std::uint32_t>(a_f2));
}
constexpr Polling<SPI_master>::Result::Bus_flag operator|=(Polling<SPI_master>::Result::Bus_flag& a_f1,
                                                           Polling<SPI_master>::Result::Bus_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr Polling<SPI_slave>::Result::Bus_flag operator|(Polling<SPI_slave>::Result::Bus_flag a_f1,
                                                         Polling<SPI_slave>::Result::Bus_flag a_f2)
{
    return static_cast<Polling<SPI_slave>::Result::Bus_flag>(static_cast<std::uint32_t>(a_f1) |
                                                             static_cast<std::uint32_t>(a_f2));
}
constexpr Polling<SPI_slave>::Result::Bus_flag operator&(Polling<SPI_slave>::Result::Bus_flag a_f1,
                                                         Polling<SPI_slave>::Result::Bus_flag a_f2)
{
    return static_cast<Polling<SPI_slave>::Result::Bus_flag>(static_cast<std::uint32_t>(a_f1) &
                                                             static_cast<std::uint32_t>(a_f2));
}
constexpr Polling<SPI_slave>::Result::Bus_flag operator|=(Polling<SPI_slave>::Result::Bus_flag& a_f1,
                                                          Polling<SPI_slave>::Result::Bus_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc