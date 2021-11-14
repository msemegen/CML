#pragma once

/*
 *   Name: Polling.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/Factory.hpp>
#include <soc/m4/stm32l4/I2C/I2C.hpp>
#include <soc/m4/stm32l4/Polling.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Polling<I2C_master> : private cml::Non_copyable
{
public:
    struct Result
    {
        enum class Bus_flag : std::uint32_t
        {
            ok               = 0x0,
            crc_error        = 0x1,
            buffer_error     = 0x2,
            arbitration_lost = 0x4,
            misplaced        = 0x8,
            nack             = 0x10,
        };

        Bus_flag bus_flag                = cml::various::get_enum_incorrect_value<Bus_flag>();
        std::size_t data_length_in_bytes = 0u;
    };

    Result transmit(std::uint8_t a_slave_address, const void* a_p_data, std::size_t a_data_size_in_bytes);
    Result transmit(std::uint8_t a_slave_address,
                    const void* a_p_data,
                    std::size_t a_data_size_in_bytes,
                    std::uint32_t a_timeout);

    Result receive(std::uint8_t a_slave_address, void* a_p_data, std::size_t a_data_size_in_bytes);
    Result
    receive(std::uint8_t a_slave_address, void* a_p_data, std::size_t a_data_size_in_bytes, std::uint32_t a_timeout);

private:
    Polling(I2C_master* a_p_I2C_master)
        : a_p_I2C(a_p_I2C_master)
    {
    }

    I2C_master* a_p_I2C;

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

template<> class Polling<I2C_slave> : private cml::Non_copyable
{
public:
    struct Result
    {
        enum class Bus_flag : std::uint32_t
        {
            ok               = 0x0,
            crc_error        = 0x1,
            buffer_error     = 0x2,
            arbitration_lost = 0x4,
            misplaced        = 0x8,
            nack             = 0x10,
        };

        Bus_flag bus_flag                = cml::various::get_enum_incorrect_value<Bus_flag>();
        std::size_t data_length_in_bytes = 0u;
    };

    Result transmit(const void* a_p_data, std::size_t a_data_size_in_bytes);
    Result transmit(const void* a_p_data, std::size_t a_data_size_in_bytes, std::uint32_t a_timeout);

    Result receive(void* a_p_data, std::size_t a_data_size_in_bytes);
    Result receive(void* a_p_data, std::size_t a_data_size_in_bytes, std::uint32_t a_timeout);

private:
    Polling(I2C_slave* a_p_I2C_slave)
        : a_p_I2C(a_p_I2C_slave)
    {
    }

    I2C_slave* a_p_I2C;

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};
constexpr Polling<I2C_master>::Result::Bus_flag operator|(Polling<I2C_master>::Result::Bus_flag a_f1,
                                                          Polling<I2C_master>::Result::Bus_flag a_f2)
{
    return static_cast<Polling<I2C_master>::Result::Bus_flag>(static_cast<std::uint32_t>(a_f1) |
                                                              static_cast<std::uint32_t>(a_f2));
}
constexpr Polling<I2C_master>::Result::Bus_flag operator&(Polling<I2C_master>::Result::Bus_flag a_f1,
                                                          Polling<I2C_master>::Result::Bus_flag a_f2)
{
    return static_cast<Polling<I2C_master>::Result::Bus_flag>(static_cast<std::uint32_t>(a_f1) &
                                                              static_cast<std::uint32_t>(a_f2));
}
constexpr Polling<I2C_master>::Result::Bus_flag operator|=(Polling<I2C_master>::Result::Bus_flag& a_f1,
                                                           Polling<I2C_master>::Result::Bus_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr Polling<I2C_slave>::Result::Bus_flag operator|(Polling<I2C_slave>::Result::Bus_flag a_f1,
                                                         Polling<I2C_slave>::Result::Bus_flag a_f2)
{
    return static_cast<Polling<I2C_slave>::Result::Bus_flag>(static_cast<std::uint32_t>(a_f1) |
                                                             static_cast<std::uint32_t>(a_f2));
}
constexpr Polling<I2C_slave>::Result::Bus_flag operator&(Polling<I2C_slave>::Result::Bus_flag a_f1,
                                                         Polling<I2C_slave>::Result::Bus_flag a_f2)
{
    return static_cast<Polling<I2C_slave>::Result::Bus_flag>(static_cast<std::uint32_t>(a_f1) &
                                                             static_cast<std::uint32_t>(a_f2));
}
constexpr Polling<I2C_slave>::Result::Bus_flag operator|=(Polling<I2C_slave>::Result::Bus_flag& a_f1,
                                                          Polling<I2C_slave>::Result::Bus_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc