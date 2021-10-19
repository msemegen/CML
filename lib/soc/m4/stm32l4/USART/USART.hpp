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

// soc
#include <soc/Handle.hpp>
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
    struct id : private cml::Non_constructible
    {
        constexpr static auto _1 = Handle<USART1_BASE> {};
        constexpr static auto _2 = Handle<USART2_BASE> {};
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        constexpr static auto _3 = Handle<USART3_BASE> {};
#endif
    };

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

public:
    USART(Handle<USART1_BASE>)
        : idx(0u)
        , p_registers(USART1)
    {
    }
    USART(Handle<USART2_BASE>)
        : idx(1u)
        , p_registers(USART2)
    {
    }
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    USART(Handle<USART3_BASE>)
        : idx(2u)
        , p_registers(USART3)
    {
    }
#endif

    ~USART();

    bool enable(const Enable_config& a_config, const Frame_format& frame_format, std::uint32_t a_timeout_ms);
    void disable();

    Enable_config get_Enable_config() const;
    Frame_format get_Frame_format() const;

    std::uint32_t get_id() const
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
    const std::uint32_t idx;
    USART_TypeDef* p_registers;
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

template<> class rcc<USART> : private cml::Non_constructible
{
public:
    enum class Clock_source : uint32_t
    {
        PCLK1,
        SYSCLK,
        HSI,
    };

    template<Clock_source, std::uint32_t peripheral_base_address>
    static void enable(Handle<peripheral_base_address>, bool a_enable_in_lp);
    template<std::uint32_t peripheral_base_address> static void disable(Handle<peripheral_base_address>);
};

template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::HSI, USART1_BASE>(Handle<USART1_BASE>, bool a_enable_in_lp);
template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::PCLK1, USART1_BASE>(Handle<USART1_BASE>, bool a_enable_in_lp);
template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::SYSCLK, USART1_BASE>(Handle<USART1_BASE>, bool a_enable_in_lp);

template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::HSI, USART2_BASE>(Handle<USART2_BASE>, bool a_enable_in_lp);
template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::PCLK1, USART2_BASE>(Handle<USART2_BASE>, bool a_enable_in_lp);
template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::SYSCLK, USART2_BASE>(Handle<USART2_BASE>, bool a_enable_in_lp);

template<> void rcc<USART>::disable<USART1_BASE>(Handle<USART1_BASE>);
template<> void rcc<USART>::disable<USART2_BASE>(Handle<USART2_BASE>);

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::HSI, USART3_BASE>(Handle<USART3_BASE>, bool a_enable_in_lp);
template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::PCLK1, USART3_BASE>(Handle<USART3_BASE>, bool a_enable_in_lp);
template<>
void rcc<USART>::enable<rcc<USART>::Clock_source::SYSCLK, USART3_BASE>(Handle<USART3_BASE>, bool a_enable_in_lp);
template<> void rcc<USART>::disable<USART3_BASE>(Handle<USART3_BASE>);
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc