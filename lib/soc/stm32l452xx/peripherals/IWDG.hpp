#pragma once

/*
 *   Name: IWDG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l452xx.h>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l452xx {
namespace peripherals {

#ifdef IWDG
#undef IWDG
#endif // IWDG

class IWDG : private cml::Non_copyable
{
public:
    enum class Prescaler : uint32_t
    {
        _4   = 0x0u,
        _8   = IWDG_PR_PR_0,
        _16  = IWDG_PR_PR_1,
        _32  = IWDG_PR_PR_1 | IWDG_PR_PR_0,
        _64  = IWDG_PR_PR_2,
        _128 = IWDG_PR_PR_2 | IWDG_PR_PR_0,
        _256 = IWDG_PR_PR_2 | IWDG_PR_PR_1
    };

    struct Window
    {
        bool enable    = false;
        uint16_t value = 0xFFFu;
    };

public:
    IWDG();
    ~IWDG();

    bool enable(Prescaler a_prescaler, uint16_t a_reload, const Window& a_window, uint32_t a_timeout);
    void disable();

    void feed();
};

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc