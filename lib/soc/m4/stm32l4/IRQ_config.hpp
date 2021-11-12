#pragma once

/*
 *   Name: IRQ_config.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

namespace soc {
namespace m4 {
namespace stm32l4 {
struct IRQ_config
{
    std::uint32_t preempt_priority = 0;
    std::uint32_t sub_priority     = 0;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc