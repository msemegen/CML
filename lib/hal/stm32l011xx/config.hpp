#pragma once

/*
    Name: config.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/frequency.hpp>
#include <common/integer.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

struct s_config
{
    struct s_usart
    {
        static constexpr common::uint32 _2_interrupt_priority = 0;

        s_usart()               = delete;
        s_usart(s_usart&&)      = delete;
        s_usart(const s_usart&) = delete;
        ~s_usart()              = delete;

        s_usart& operator = (s_usart&)       = delete;
        s_usart& operator = (const s_usart&) = delete;
    };

    struct s_mcu
    {
        static constexpr common::uint8 device_id_length = 12;

        s_mcu()             = delete;
        s_mcu(s_mcu&&)      = delete;
        s_mcu(const s_mcu&) = delete;
        ~s_mcu()            = delete;

        s_mcu& operator = (s_mcu&)       = delete;
        s_mcu& operator = (const s_mcu&) = delete;
    };

    struct s_clock
    {
        static constexpr common::uint32 hsi_frequency_hz = common::MHz(16u);
        static constexpr common::uint32 lsi_frequency_hz = common::kHz(37u);

        s_clock()               = delete;
        s_clock(s_clock&&)      = delete;
        s_clock(const s_clock&) = delete;
        ~s_clock()              = delete;

        s_clock& operator = (s_clock&)       = delete;
        s_clock& operator = (const s_clock&) = delete;
    };

    s_config()                = delete;
    s_config(s_config&&)      = delete;
    s_config(const s_config&) = delete;
    ~s_config()               = delete;

    s_config& operator = (s_config&)       = delete;
    s_config& operator = (const s_config&) = delete;
};

} // namesapace stm32l011xx
} // namesapace hal
} // namesapace cml