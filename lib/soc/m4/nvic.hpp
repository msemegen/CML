#pragma once

/*
 *   Name: nvic.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// external
#ifdef STM32L4
#include <stm32l4xx.h>
#endif

// cml
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {

class nvic
{
public:
    enum class Mode
    {
        disabled,
        enabled
    };

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

public:
    static void set_config(const Config& a_config)
    {
        cml_assert(cml::various::get_enum_incorrect_value<Config::Grouping>() != a_config.grouping);

        NVIC_SetPriorityGrouping(static_cast<uint32_t>(a_config.grouping));
        __set_BASEPRI(a_config.base_priority << 0x4u);
    }

    static void set_mode(Mode a_mode)
    {
        static uint32_t primask = 0;

        switch (a_mode)
        {
            case Mode::enabled: {
                __set_PRIMASK(primask);
            }
            break;

            case Mode::disabled: {
                primask = __get_PRIMASK();
                __disable_irq();
            }
            break;
        }
    }

    static Config get_config()
    {
        return { static_cast<Config::Grouping>(NVIC_GetPriorityGrouping()), __get_BASEPRI() };
    }

    static Mode get_mode()
    {
        uint32_t cpsr;
        asm volatile(" mrs  %0, cpsr" : "=r"(cpsr) : /* no inputs */);

        return static_cast<Mode>(cml::bit::is(cpsr, 0x7u));
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