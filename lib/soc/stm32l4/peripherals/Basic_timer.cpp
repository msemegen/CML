/*
 *   Name: Basic_timer.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/stm32l4/peripherals/Basic_timer.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

// externals
#include <stm32l4xx.h>

namespace {

using namespace cml;
using namespace soc::stm32l4::peripherals;

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

struct Controller
{
    using Enable_function  = void (*)(uint32_t a_irq_priority);
    using Disable_function = void (*)();

    TIM_TypeDef* p_registers    = nullptr;
    Basic_timer* p_timer_handle = nullptr;

    Enable_function enable   = nullptr;
    Disable_function disable = nullptr;
};

void tim6_enable(uint32_t a_irq_priority)
{
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN);
    NVIC_SetPriority(TIM6_DAC_IRQn, a_irq_priority);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void tim6_disable()
{
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN);
};

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
void tim7_enable(uint32_t a_irq_priority)
{
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM7EN);
    NVIC_SetPriority(TIM7_IRQn, a_irq_priority);
    NVIC_EnableIRQ(TIM7_IRQn);
}

void tim7_disable()
{
    NVIC_DisableIRQ(TIM7_IRQn);
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM7EN);
};
#endif

Controller controllers[] = { { TIM6, nullptr, tim6_enable, tim6_disable },
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
                             { TIM7, nullptr, tim7_enable, tim7_disable }
#endif
};

TIM_TypeDef* get_timer_registers(Basic_timer::Id a_id)
{
    return controllers[static_cast<uint32_t>(a_id)].p_registers;
}

#endif

} // namespace

extern "C" {
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

void TIM6_DAC_IRQHandler()
{
    interrupt_handler(controllers[0].p_timer_handle);
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
void TIM7_IRQHandler()
{
    interrupt_handler(controllers[1].p_timer_handle);
}
#endif

#endif
}

namespace soc {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

void interrupt_handler(Basic_timer* a_p_this)
{
    cml_assert(nullptr != a_p_this->overload_callback.function);
    a_p_this->overload_callback.function(a_p_this->overload_callback.p_user_data);

    get_timer_registers(a_p_this->id)->SR = 0;
}

void Basic_timer::enable(uint16_t a_prescaler, uint16_t a_auto_reload, uint32_t a_irq_priority)
{
    controllers[static_cast<uint32_t>(this->id)].enable(a_irq_priority);
    controllers[static_cast<uint32_t>(this->id)].p_timer_handle = this;

    get_timer_registers(this->id)->SR  = 0;
    get_timer_registers(this->id)->CNT = 0;
    get_timer_registers(this->id)->PSC = a_prescaler;
    get_timer_registers(this->id)->ARR = a_auto_reload;

    get_timer_registers(this->id)->CR1 = TIM_CR1_UIFREMAP | TIM_CR1_ARPE;
}

void Basic_timer::disable()
{
    get_timer_registers(this->id)->CR1 = 0;
    controllers[static_cast<uint32_t>(this->id)].disable();
    controllers[static_cast<uint32_t>(this->id)].p_timer_handle = nullptr;
}

void Basic_timer::start()
{
    bit_flag::set(&(get_timer_registers(this->id)->CR1), TIM_CR1_CEN);
}

void Basic_timer::stop()
{
    bit_flag::clear(&(get_timer_registers(this->id)->CR1), TIM_CR1_CEN);
}

void Basic_timer::register_overload_callback(const Overload_callback& a_callback)
{
    this->overload_callback             = a_callback;
    get_timer_registers(this->id)->DIER = TIM_DIER_UIE;
}

void Basic_timer::unregister_overload_callback()
{
    this->overload_callback             = { nullptr, nullptr };
    get_timer_registers(this->id)->DIER = 0;
}

bool Basic_timer::is_overload_event() const
{
    cml_assert(nullptr == this->overload_callback.function);

    if (true == bit_flag::is(get_timer_registers(this->id)->SR, TIM_SR_UIF))
    {
        get_timer_registers(this->id)->SR = 0;
        return true;
    }

    return false;
}

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc
#endif