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
#include <soc/m4/stm32l4/mcu/mcu.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
struct cycles_counter : private cml::Non_constructible
{
    static void reset()
    {
        cml_assert(true == mcu::is_DWT_active());

        DWT->CYCCNT = 0;
    }

    static std::uint32_t get()
    {
        cml_assert(true == mcu::is_DWT_active());

        return DWT->CYCCNT;
    }
};
} // namespace stm32l4
} // namespace m4
} // namespace soc