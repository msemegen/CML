#pragma once

/*
    Name: config.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

struct s_config
{
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