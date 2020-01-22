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

struct config
{
    struct usart
    {
        static constexpr common::uint32 _2_interrupt_priority = 0;

        usart()             = delete;
        usart(usart&&)      = delete;
        usart(const usart&) = delete;
        ~usart()            = delete;

        usart& operator = (usart&)       = delete;
        usart& operator = (const usart&) = delete;
    };

    struct mcu
    {
        static constexpr common::uint8 device_id_length = 12;

        mcu()           = delete;
        mcu(mcu&&)      = delete;
        mcu(const mcu&) = delete;
        ~mcu()          = delete;

        mcu& operator = (mcu&)       = delete;
        mcu& operator = (const mcu&) = delete;
    };

    struct clock
    {
        static constexpr common::uint32 hsi_frequency_hz = common::MHz(16u);
        static constexpr common::uint32 lsi_frequency_hz = common::kHz(37u);

        clock()               = delete;
        clock(clock&&)      = delete;
        clock(const clock&) = delete;
        ~clock()              = delete;

        clock& operator = (clock&)       = delete;
        clock& operator = (const clock&) = delete;
    };

    config()              = delete;
    config(config&&)      = delete;
    config(const config&) = delete;
    ~config()             = delete;

    config& operator = (config&)       = delete;
    config& operator = (const config&) = delete;
};

} // namesapace stm32l011xx
} // namesapace hal
} // namesapace cml