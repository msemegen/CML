#pragma once

/*
 *   Name: IRQ.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// cml
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
struct IRQ
{
    bool active                    = false;
    std::uint32_t preempt_priority = 0;
    std::uint32_t sub_priority     = 0;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc