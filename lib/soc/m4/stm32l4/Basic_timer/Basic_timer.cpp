/*
 *   Name: Basic_timer.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/Basic_timer/Basic_timer.hpp>

// soc
#include <soc/m4/Interrupt_guard.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void Basic_timer_interrupt_handler(Basic_timer* a_p_this)
{
    a_p_this->callback.function(a_p_this, a_p_this->callback.p_user_data);

    static_cast<TIM_TypeDef*>(*(a_p_this))->SR = 0;
}

void Basic_timer::enable(const Enable_config& a_config)
{
    cml_assert(true == this->is_ceated());

    cml_assert(various::get_enum_incorrect_value<Enable_config::Mode>() != a_config.mode);
    cml_assert(various::get_enum_incorrect_value<Enable_config::Autoreload_preload>() != a_config.auto_reload_preload);

    this->p_registers->CNT = 0u;
    this->p_registers->PSC = a_config.prescaler;
    this->p_registers->ARR = a_config.auto_reload;

    this->p_registers->CR1 = TIM_CR1_UIFREMAP | TIM_CR1_ARPE;
    this->p_registers->SR  = 0u;
}

void Basic_timer::disable()
{
    cml_assert(true == this->is_ceated());

    this->p_registers->CR1 = 0x0u;
}

void Basic_timer::start()
{
    cml_assert(true == this->is_ceated());

    bit_flag::set(&(this->p_registers->CR1), TIM_CR1_CEN);
}

void Basic_timer::stop()
{
    cml_assert(true == this->is_ceated());

    bit_flag::clear(&(this->p_registers->CR1), TIM_CR1_CEN);
}

void Basic_timer::Interrupt::enable(const IRQ_config& a_irq_config)
{
    this->set_irq_context();

    NVIC_SetPriority(
        this->p_timer->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_timer->irqn);
}

void Basic_timer::Interrupt::disable()
{
    this->unregister_callback();

    NVIC_DisableIRQ(this->p_timer->irqn);

    this->clear_irq_context();
}

void Basic_timer::Interrupt::register_callback(const Callback& a_callback)
{
    cml_assert(nullptr != a_callback.function);

    Interrupt_guard guard;

    this->p_timer->callback                           = a_callback;
    static_cast<TIM_TypeDef*>(*(this->p_timer))->DIER = TIM_DIER_UIE;
}

void Basic_timer::Interrupt::unregister_callback()
{
    Interrupt_guard guard;

    static_cast<TIM_TypeDef*>(*(this->p_timer))->DIER = 0x0u;

    this->p_timer->callback = { nullptr, nullptr };
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif