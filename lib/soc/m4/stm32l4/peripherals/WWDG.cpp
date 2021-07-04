/*
 *   Name: WWDG.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/peripherals/WWDG.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

namespace {

using namespace soc::m4::stm32l4::peripherals;

uint16_t reload = 0;
bool created    = false;

WWDG::Early_wakeup_callback callback;

} // namespace

extern "C" {

#define WWDG_T ((WWDG_TypeDef*)WWDG_BASE)

void WWDG_IRQHandler()
{
    if (nullptr != callback.function)
    {
        callback.function(callback.p_user_data);
    }
}

} // extern C

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

using namespace cml;

WWDG::WWDG()
{
    cml_assert(false == created);
    created = true;
}

void WWDG::enable(Prescaler a_prescaler, uint16_t a_reload, uint16_t a_window, uint16_t a_irq_priority)
{
    WWDG_T->CR  = (WWDG_CR_WDGA | a_reload);
    WWDG_T->CFR = static_cast<uint32_t>(a_prescaler) | a_window;

    NVIC_SetPriority(WWDG_IRQn, a_irq_priority);
    NVIC_EnableIRQ(WWDG_IRQn);
}

void WWDG::register_early_wakeup_callback(const Early_wakeup_callback& a_callback)
{
    Interrupt_guard gaurd;

    callback = a_callback;

    bit_flag::set(&(WWDG_T->CFR), WWDG_CFR_EWI);
}

void WWDG::unregister_early_wakeup_callback()
{
    Interrupt_guard guard;

    bit_flag::clear(&(WWDG_T->CFR), WWDG_CFR_EWI);
    callback = { nullptr, nullptr };
}

void WWDG::feed()
{
    WWDG_T->CR = reload;
}

} // namespace peripherals
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4