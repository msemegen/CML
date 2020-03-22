#pragma once

//externals
#include <stm32l452xx.h>

//cml
#include <common/integer.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class wwdg
{
public:

    enum class Prescaler : common::uint32
    {
        _1 = 0,
        _2 = WWDG_CFR_WDGTB_0,
        _4 = WWDG_CFR_WDGTB_1,
        _8 = WWDG_CFR_WDGTB_0 | WWDG_CFR_WDGTB_1
    };

    struct Callback
    {
        using Function = void(*)(void* a_p_user_data);

        Function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

public:

    wwdg()            = delete;
    wwdg(wwdg&&)      = delete;
    wwdg(const wwdg&) = delete;

    wwdg& operator = (wwdg&&)      = delete;
    wwdg& operator = (const wwdg&) = delete;

    static void enable(Prescaler a_prescaler, common::uint16 a_reload, common::uint16 a_window, common::uint16 a_irq_priority);
    static void register_early_wakeup_callback(const Callback& a_callback);
    static void unregister_early_wakeup_callback();

    static void feed();
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml