/*
 *   Name: Interrupt.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/WWDG/Interrupt.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {

using namespace soc::m4::stm32l4;

Interrupt<WWDG>* p_WWDG = nullptr;

#define WWDG_T ((WWDG_TypeDef*)WWDG_BASE)

} // namespace

extern "C" {

using namespace soc::m4::stm32l4;

void WWDG_IRQHandler()
{
    WWDG_interrupt_handler(p_WWDG);
}
}

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void WWDG_interrupt_handler(Interrupt<WWDG>* a_p_this)
{
    cml_assert(nullptr != a_p_this);
    cml_assert(nullptr != a_p_this->early_wakeup_callback.function);

    a_p_this->early_wakeup_callback.function(a_p_this->early_wakeup_callback.p_user_data);
}

Interrupt<WWDG>::Interrupt()
{
    cml_assert(nullptr == p_WWDG);
    p_WWDG = this;
}

Interrupt<WWDG>::~Interrupt()
{
    p_WWDG = nullptr;
}

void Interrupt<WWDG>::enable(const IRQ& a_irq)
{
    cml_assert(true == a_irq.active);

    NVIC_SetPriority(IRQn_Type::WWDG_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq.preempt_priority, a_irq.sub_priority));
    NVIC_EnableIRQ(IRQn_Type::RNG_IRQn);
}

void Interrupt<WWDG>::disable()
{
    NVIC_DisableIRQ(IRQn_Type::RNG_IRQn);
}

void Interrupt<WWDG>::register_callback(const Early_wakeup_callback& a_callback)
{
    Interrupt_guard gaurd;

    if (nullptr != a_callback.function)
    {
        this->early_wakeup_callback = a_callback;
        bit_flag::set(&(WWDG_T->CFR), WWDG_CFR_EWI);
    }
    else
    {
        bit_flag::clear(&(WWDG_T->CFR), WWDG_CFR_EWI);
        this->early_wakeup_callback = { nullptr, nullptr };
    }
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif