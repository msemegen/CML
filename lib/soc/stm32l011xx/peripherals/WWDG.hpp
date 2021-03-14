#pragma once

/*
 *   Name: WWDG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l011xx.h>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

#ifdef WWDG
#undef WWDG
#endif // IWDG

class WWDG
{
public:
    enum class Prescaler : uint32_t
    {
        _1 = 0,
        _2 = WWDG_CFR_WDGTB_0,
        _4 = WWDG_CFR_WDGTB_1,
        _8 = WWDG_CFR_WDGTB_0 | WWDG_CFR_WDGTB_1
    };

    struct Callback
    {
        using Function = void (*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    WWDG();

    void enable(Prescaler a_prescaler, uint16_t a_reload, uint16_t a_window, uint16_t a_irq_priority);

    void register_early_wakeup_callback(const Callback& a_callback);
    void unregister_early_wakeup_callback();

    void feed();
};

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc