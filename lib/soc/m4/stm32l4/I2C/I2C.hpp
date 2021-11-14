#pragma once

/*
 *   Name: I2C.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Factory.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class I2C : private cml::Non_copyable
{
public:
    std::uint32_t get_idx() const
    {
        return this->idx;
    }

    operator I2C_TypeDef*()
    {
        return this->p_registers;
    }

    operator const I2C_TypeDef*() const
    {
        return this->p_registers;
    }

protected:
    I2C(std::size_t a_idx, I2C_TypeDef* a_p_registers)
        : idx(a_idx)
        , p_registers(a_p_registers)
    {
    }

    const std::uint32_t idx;
    I2C_TypeDef* p_registers;
};

class I2C_master : public I2C
{
public:
    struct Enable_config
    {
        enum class Analog_filter : std::uint32_t
        {
            disabled,
            enabled
        };

        enum class Fast_plus : std::uint32_t
        {
            disabled,
            enabled
        };

        enum class Crc : std::uint32_t
        {
            disabled,
            enabled
        };

        Analog_filter analog_filter = cml::various::get_enum_incorrect_value<Analog_filter>();
        Fast_plus fast_plus         = cml::various::get_enum_incorrect_value<Fast_plus>();
        Crc crc                     = cml::various::get_enum_incorrect_value<Crc>();
        std::uint32_t timings       = 0;
    };

    ~I2C_master();

    void enable(const Enable_config& a_config);
    void disable();

    Enable_config get_Enable_config() const;

private:
    I2C_master(std::size_t a_idx, I2C_TypeDef* a_p_registers)
        : I2C(a_idx, a_p_registers)
    {
    }

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

class I2C_slave : public I2C
{
public:
    struct Enable_config
    {
        enum class Analog_filter : std::uint32_t
        {
            disabled,
            enabled,
        };

        enum class Fast_plus : std::uint32_t
        {
            disabled,
            enabled,
        };

        enum class Crc : std::uint32_t
        {
            disabled,
            enabled,
        };

        Analog_filter analog_filter = cml::various::get_enum_incorrect_value<Analog_filter>();
        Fast_plus fast_plus         = cml::various::get_enum_incorrect_value<Fast_plus>();
        Crc crc                     = cml::various::get_enum_incorrect_value<Crc>();
        std::uint32_t timings       = 0;
        std::uint16_t address       = 0;
    };

public:
    ~I2C_slave();

    void enable(const Enable_config& a_config);
    void disable();

    Enable_config get_Enable_config() const;

private:
    I2C_slave(std::size_t a_idx, I2C_TypeDef* a_p_registers)
        : I2C(a_idx, a_p_registers)
    {
    }

private:
    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

template<std::size_t id> class rcc<I2C, id> : private cml::Non_constructible
{
public:
    enum class Clock_source : uint32_t
    {
        PCLK1  = 0,
        SYSCLK = 1,
        HSI    = 2
    };

    template<Clock_source> static void enable(bool a_enable_in_lp) = delete;
    static void disable()                                          = delete;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc