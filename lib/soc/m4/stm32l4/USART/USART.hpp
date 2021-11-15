#pragma once

/*
 *   Name: USART.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// std
#include <cstdint>
#include <tuple>

// soc
#include <soc/Factory.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class USART : private cml::Non_copyable
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

        enum class Flow_control_flag : std::uint32_t
        {
            none            = 0x0u,
            request_to_send = USART_CR3_RTSE,
            clear_to_send   = USART_CR3_CTSE,
        };

        enum Sampling_method : std::uint32_t
        {
            three_sample_bit = 0,
            one_sample_bit   = USART_CR3_ONEBIT,
        };

        enum class Mode_flag : std::uint32_t
        {
            tx = USART_CR1_TE,
            rx = USART_CR1_RE,
        };

        std::uint32_t baud_rate         = 0;
        std::uint32_t clock_freq_Hz     = 0;
        Oversampling oversampling       = cml::various::get_enum_incorrect_value<Oversampling>();
        Stop_bits stop_bits             = cml::various::get_enum_incorrect_value<Stop_bits>();
        Flow_control_flag flow_control  = cml::various::get_enum_incorrect_value<Flow_control_flag>();
        Sampling_method sampling_method = cml::various::get_enum_incorrect_value<Sampling_method>();
        Mode_flag mode                  = cml::various::get_enum_incorrect_value<Mode_flag>();
    };

    struct Frame_format
    {
        enum class Word_length : uint32_t
        {
            _7_bit = USART_CR1_M1,
            _8_bit = 0x0u,
            _9_bit = USART_CR1_M0,
        };

        enum class Parity : uint32_t
        {
            none = 0x0u,
            even = USART_CR1_PCE,
            odd  = USART_CR1_PCE | USART_CR1_PS,
        };

        Word_length word_length = cml::various::get_enum_incorrect_value<Word_length>();
        Parity parity           = cml::various::get_enum_incorrect_value<Parity>();
    };

    USART()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
    {
    }
    ~USART();

    bool enable(const Enable_config& a_config, const Frame_format& frame_format, std::uint32_t a_timeout_ms);
    void disable();

    Enable_config get_Enable_config() const;
    Frame_format get_Frame_format() const;

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
    USART(std::size_t a_idx, USART_TypeDef* a_p_registers)
        : idx(a_idx)
        , p_registers(a_p_registers)
    {
    }

    const std::uint32_t idx;
    USART_TypeDef* p_registers;

    template<typename Periph_t, std::size_t id> friend class soc::Factory;
};

constexpr USART::Enable_config::Flow_control_flag operator|(USART::Enable_config::Flow_control_flag a_f1,
                                                            USART::Enable_config::Flow_control_flag a_f2)
{
    return static_cast<USART::Enable_config::Flow_control_flag>(static_cast<uint32_t>(a_f1) |
                                                                static_cast<uint32_t>(a_f2));
}

constexpr USART::Enable_config::Flow_control_flag operator&(USART::Enable_config::Flow_control_flag a_f1,
                                                            USART::Enable_config::Flow_control_flag a_f2)
{
    return static_cast<USART::Enable_config::Flow_control_flag>(static_cast<uint32_t>(a_f1) &
                                                                static_cast<uint32_t>(a_f2));
}

constexpr USART::Enable_config::Flow_control_flag operator|=(USART::Enable_config::Flow_control_flag& a_f1,
                                                             USART::Enable_config::Flow_control_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr USART::Enable_config::Mode_flag operator|(USART::Enable_config::Mode_flag a_f1,
                                                    USART::Enable_config::Mode_flag a_f2)
{
    return static_cast<USART::Enable_config::Mode_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr USART::Enable_config::Mode_flag operator&(USART::Enable_config::Mode_flag a_f1,
                                                    USART::Enable_config::Mode_flag a_f2)
{
    return static_cast<USART::Enable_config::Mode_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr USART::Enable_config::Mode_flag operator|=(USART::Enable_config::Mode_flag& a_f1,
                                                     USART::Enable_config::Mode_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

template<std::size_t id> class rcc<USART, id> : private cml::Non_constructible
{
public:
    enum class Clock_source : uint32_t
    {
        PCLK1,
        SYSCLK,
        HSI,
    };

    template<Clock_source> static void enable(bool a_enable_in_lp) = delete;
    static void disable()                                          = delete;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc