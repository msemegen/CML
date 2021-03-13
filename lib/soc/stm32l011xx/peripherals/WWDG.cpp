/*
 *   Name: WWDG.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/peripherals/WWDG.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l011xx::peripherals;

uint16_t reload = 0;

WWDG::Callback callback;

bool created = false;

} // namespace

extern "C" {

void WWDG_IRQHandler()
{
    if (nullptr != callback.function)
    {
        callback.function(callback.p_user_data);
    }
}

} // extern C

namespace soc {
namespace stm32l011xx {
namespace peripherals {

using namespace cml;

WWDG::WWDG()
{
    cml_assert(false == created);
    created = true;
}

void WWDG::enable(Prescaler a_prescaler, uint16_t a_reload, uint16_t a_window, uint16_t a_irq_priority)
{
#ifndef WWDG
#define WWDG ((WWDG_TypeDef*)WWDG_BASE)
#endif

    WWDG->CR  = (WWDG_CR_WDGA | a_reload);
    WWDG->CFR = static_cast<uint32_t>(a_prescaler) | a_window;

#ifdef WWDG
#undef WWDG
#endif

    NVIC_SetPriority(WWDG_IRQn, a_irq_priority);
    NVIC_EnableIRQ(WWDG_IRQn);
}

void WWDG::register_early_wakeup_callback(const Callback& a_callback)
{
    Interrupt_guard guard;

    callback = a_callback;

#ifndef WWDG
#define WWDG ((WWDG_TypeDef*)WWDG_BASE)
#endif

    bit_flag::set(&(WWDG->CFR), WWDG_CFR_EWI);

#ifdef WWDG
#undef WWDG
#endif
}

void WWDG::unregister_early_wakeup_callback()
{
    Interrupt_guard guard;

#ifndef WWDG
#define WWDG ((WWDG_TypeDef*)WWDG_BASE)
#endif

    bit_flag::clear(&(WWDG->CFR), WWDG_CFR_EWI);

#ifdef WWDG
#undef WWDG
#endif

    callback = { nullptr, nullptr };
}

void WWDG::feed()
{
#ifndef WWDG
#define WWDG ((WWDG_TypeDef*)WWDG_BASE)
#endif

    WWDG->CR = reload;

#ifdef WWDG
#undef WWDG
#endif
}

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx