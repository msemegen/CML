#pragma once

/*
 *   Name: IRQ_config.hpp
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
struct IRQ_config
{
    enum class Mode : std::uint32_t
    {
        enabled,
        disabled
    };
    Mode mode                      = cml::various::get_enum_incorrect_value<Mode>();
    std::uint32_t preempt_priority = 0;
    std::uint32_t sub_priority     = 0;
};
} // namespace m4
} // namespace soc