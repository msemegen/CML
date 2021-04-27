#pragma once

/*
    Name: IWDG.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// externals
#include <stm32l011xx.h>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

#ifdef IWDG
#undef IWDG
#endif // IWDG

class IWDG
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
} // namespace stm32l011xx
} // namespace soc