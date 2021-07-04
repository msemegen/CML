#pragma once

/*
 *   Name: nvic.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <stdint.h>

// externals
#include <stm32l4xx.h>

// cml
#include <cml/various.hpp>

namespace soc {
namespace m4 {

class nvic
{
public:
    struct Config
    {
        enum class Grouping : uint32_t
        {
            _0 = 0x7,
            _1 = 0x6,
            _2 = 0x5,
            _3 = 0x4,
            _4 = 0x3,
        };

        Grouping grouping      = cml::various::get_enum_incorrect_value<Grouping>();
        uint32_t base_priority = 0;
    };

    static void set_config(const Config& a_config)
    {
        NVIC_SetPriorityGrouping(static_cast<uint32_t>(a_config.grouping));
        __set_BASEPRI(a_config.base_priority);
    }

private:
    nvic()            = delete;
    nvic(const nvic&) = delete;
    nvic(nvic&&)      = delete;
    ~nvic()           = delete;

    nvic& operator=(const nvic&) = delete;
    nvic& operator=(nvic&&) = delete;
};

} // namespace m4
} // namespace soc