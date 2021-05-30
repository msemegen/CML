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

    TIM_TypeDef* p_registers = nullptr;

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

Controller controllers[] = { { TIM6, tim6_enable, tim6_disable },
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
                             { TIM7, tim7_enable, tim7_disable }
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

    void TIM6_IRQHandler() {}



#endif
}

namespace soc {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

void Basic_timer::enable(uint16_t a_prescaler, uint16_t a_auto_reload, uint32_t a_irq_priority)
{
    controllers[static_cast<uint32_t>(this->id)].enable(a_irq_priority);

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
}

void Basic_timer::start()
{
    bit_flag::set(&(get_timer_registers(this->id)->CR1), TIM_CR1_CEN);
}

void Basic_timer::stop()
{
    bit_flag::clear(&(get_timer_registers(this->id)->CR1), TIM_CR1_CEN);
}

void Basic_timer::register_overload_callback(const Overload_callback& a_callback) {}

void Basic_timer::unregister_overload_callback() {}

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc
#endif