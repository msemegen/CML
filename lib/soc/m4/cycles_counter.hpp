#pragma once

/*
 *   Name: cycles_counter.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// soc
#ifdef STM32L4
#include <soc/m4/stm32l4/mcu/mcu.hpp>
#endif

// cml
#include <cml/Non_constructible.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
struct cycles_counter : private cml::Non_constructible
{
    static void reset()
    {
#ifdef STM32L4
        cml_assert(true == stm32l4::mcu::is_DWT_active());
#endif

        DWT->CYCCNT = 0;
    }

    static std::uint32_t get()
    {
#ifdef STM32L4
        cml_assert(true == stm32l4::mcu::is_DWT_active());
#endif

        return DWT->CYCCNT;
    }
};
} // namespace m4
} // namespace soc