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
namespace stm32l452xx{

struct config
{
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
        static constexpr common::uint32 lsi_frequency_hz = common::kHz(32u);

        clock()             = delete;
        clock(clock&&)      = delete;
        clock(const clock&) = delete;
        ~clock()            = delete;

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

} // namespace stm32l452xx
} // namespace hal
} // namespace cml
