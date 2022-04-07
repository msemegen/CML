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
        cml_assert(stm32l4::mcu::DWT_mode::enabled == stm32l4::mcu::get_DWT_mode());
#endif

        DWT->CYCCNT = 0;
    }

    static std::uint32_t get()
    {
#ifdef STM32L4
        cml_assert(stm32l4::mcu::DWT_mode::enabled == stm32l4::mcu::get_DWT_mode());
#endif

        return DWT->CYCCNT;
    }
};
} // namespace m4
} // namespace soc