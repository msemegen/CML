#pragma once

/*
    Name: wwdg.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

//externals
#include <stm32l452xx.h>

namespace soc {
namespace stm32l452xx {
namespace system {

class wwdg
{
public:

    enum class Prescaler : uint32_t
    {
        _1 = 0,
        _2 = WWDG_CFR_WDGTB_0,
        _4 = WWDG_CFR_WDGTB_1,
        _8 = WWDG_CFR_WDGTB_0 | WWDG_CFR_WDGTB_1
    };

    struct Early_wakeup_callback
    {
        using Function = void(*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:

    wwdg()            = delete;
    wwdg(wwdg&&)      = delete;
    wwdg(const wwdg&) = delete;

    wwdg& operator = (wwdg&&)      = delete;
    wwdg& operator = (const wwdg&) = delete;

    static void enable(Prescaler a_prescaler,
                       uint16_t a_reload,
                       uint16_t a_window,
                       uint16_t a_irq_priority);

    static void register_early_wakeup_callback(const Early_wakeup_callback& a_callback);
    static void unregister_early_wakeup_callback();

    static void feed();
};

} // namespace system
} // namespace stm32l452xx
} // namespace soc