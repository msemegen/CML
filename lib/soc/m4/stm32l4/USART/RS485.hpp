#pragma once

/*
 *   Name: RS485.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <tuple>

// externals
#include <stm32l4xx.h>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class RS485 : private cml::Non_copyable
{
public:
    struct Enable_config
    {
        enum class Oversampling : std::uint32_t
        {
            _8  = USART_CR1_OVER8,
            _16 = 0,
        };

        enum class Stop_bits : std::uint32_t
        {
            _0_5 = USART_CR2_STOP_0,
            _1   = 0x0u,
            _1_5 = USART_CR2_STOP_0 | USART_CR2_STOP_1,
            _2   = USART_CR2_STOP_1,
        };

        std::uint32_t baud_rate     = 0;
        std::uint32_t clock_freq_Hz = 0;
        Oversampling oversampling   = cml::various::get_enum_incorrect_value<Oversampling>();
        Stop_bits stop_bits         = cml::various::get_enum_incorrect_value<Stop_bits>();
        std::uint8_t address        = 0;
    };

public:
    ~RS485()
    {
        this->disable();
    }

    bool enable(const Enable_config& a_config, std::uint32_t a_timeout);
    void disable();

    std::uint32_t get_idx() const
    {
        return this->idx;
    }

    operator USART_TypeDef*()
    {
        return this->p_registers;
    }

    operator const USART_TypeDef*() const
    {
        return this->p_registers;
    }

private:
    RS485(std::size_t a_idx, USART_TypeDef* a_p_USART)
        : idx(a_idx)
        , p_registers(a_p_USART)
    {
    }

private:
    const std::uint32_t idx;
    USART_TypeDef* p_registers;

private:
    template<typename Periph_t, std::size_t id> friend class Factory;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc