#pragma once

/*
    Name: WWDG.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

#ifdef WWDG
#undef WWDG
#endif

class WWDG : private cml::Non_copyable
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
        using Function = void (*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    WWDG();

    void enable(Prescaler a_prescaler, uint16_t a_reload, uint16_t a_window, uint16_t a_irq_priority);

    void register_early_wakeup_callback(const Early_wakeup_callback& a_callback);
    void unregister_early_wakeup_callback();

    void feed();
};

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc