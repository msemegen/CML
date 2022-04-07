/*
 *   Name: WWDG.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/WWDG/WWDG.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

// soc
#include <soc/m4/Interrupt_guard.hpp>

namespace {
using namespace soc::m4::stm32l4;

#define WWDG_T ((WWDG_TypeDef*)WWDG_BASE)

WWDG* irq_context[1] = { nullptr };
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

void WWDG::enable(Prescaler a_prescaler, uint16_t a_reload, uint16_t a_window)
{
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    WWDG_T->CR  = (WWDG_CR_WDGA | a_reload);
    WWDG_T->CFR = static_cast<uint32_t>(a_prescaler) | a_window;

    this->reload = a_reload;
}

void WWDG::feed()
{
    WWDG_T->CR = this->reload;
}

bool WWDG::is_enabled() const
{
    return bit_flag::is(WWDG_T->CR, WWDG_CR_WDGA);
}

void WWDG::Interrupt::enable(const IRQ_config& a_irq_config)
{
    cml_assert(nullptr != this->p_WWDG);

    irq_context[0] = this->p_WWDG;

    NVIC_SetPriority(
        IRQn_Type::WWDG_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(IRQn_Type::RNG_IRQn);
}

void WWDG::Interrupt::disable()
{
    cml_assert(nullptr != this->p_WWDG);

    NVIC_DisableIRQ(IRQn_Type::RNG_IRQn);

    irq_context[0] = nullptr;
}

void WWDG::Interrupt::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != this->p_WWDG);
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard gaurd;

    this->p_WWDG->callback = a_callback;
    bit_flag::set(&(WWDG_T->CFR), WWDG_CFR_EWI);
}

void WWDG::Interrupt::unregister_callback()
{
    cml_assert(nullptr != this->p_WWDG);

    bit_flag::clear(&(WWDG_T->CFR), WWDG_CFR_EWI);
    this->p_WWDG->callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif