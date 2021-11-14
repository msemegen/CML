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

// SIC! GCC bug?
#ifdef __GNUC__
namespace soc {
namespace m4 {
namespace stm32l4 {
extern void WWDG_interrupt_handler();
} // namespace stm32l4
} // namespace m4
} // namespace soc
#endif


namespace {

using namespace soc::m4::stm32l4;

Interrupt<WWDG>* irq_context[1] = { nullptr };

#define WWDG_T ((WWDG_TypeDef*)WWDG_BASE)

} // namespace

extern "C" {

using namespace soc::m4::stm32l4;

void WWDG_IRQHandler()
{
    WWDG_interrupt_handler();
}
}

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void WWDG_interrupt_handler()
{
    cml_assert(nullptr != irq_context[0]);

    if (nullptr != irq_context[0]->callback.function)
    {
        irq_context[0]->callback.function(irq_context[0]->callback.p_user_data);
    }
}

void Interrupt<WWDG>::enable(const IRQ_config& a_irq_config)
{
    irq_context[0] = this;

    NVIC_SetPriority(
        IRQn_Type::WWDG_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(IRQn_Type::RNG_IRQn);
}

void Interrupt<WWDG>::disable()
{
    NVIC_DisableIRQ(IRQn_Type::RNG_IRQn);

    irq_context[0] = nullptr;
}

void Interrupt<WWDG>::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard gaurd;

    this->callback = a_callback;
    bit_flag::set(&(WWDG_T->CFR), WWDG_CFR_EWI);
}

void Interrupt<WWDG>::unregister_callback()
{
    bit_flag::clear(&(WWDG_T->CFR), WWDG_CFR_EWI);
    this->callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif