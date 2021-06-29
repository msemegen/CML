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
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

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
    void feed();
};

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace m4
} // namespace soc