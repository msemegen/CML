/*
 *   Name: Basic_timer.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/peripherals/Basic_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

// externals
#include <stm32l4xx.h>

namespace {

using namespace cml;
using namespace soc::m4::stm32l4::peripherals;

struct Controller
{
    TIM_TypeDef* p_registers    = nullptr;
    Basic_timer* p_timer_handle = nullptr;
};

Controller controllers[] = { { TIM6, nullptr },
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
                             { TIM7, nullptr }
#endif
};

TIM_TypeDef* get_timer_registers(Basic_timer::Id a_id)
{
    return controllers[static_cast<uint32_t>(a_id)].p_registers;
}

} // namespace

extern "C" {

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

}

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

void interrupt_handler(Basic_timer* a_p_this)
{
    cml_assert(nullptr != a_p_this->overload_callback.function);
    a_p_this->overload_callback.function(a_p_this->overload_callback.p_user_data);

    get_timer_registers(a_p_this->id)->SR = 0;
}

void Basic_timer::enable(const Config& a_config, uint32_t a_irq_priority)
{
    controllers[static_cast<uint32_t>(this->id)].p_timer_handle = this;

    get_timer_registers(this->id)->CNT = 0;
    get_timer_registers(this->id)->PSC = a_config.prescaler;
    get_timer_registers(this->id)->ARR = a_config.auto_reload;

    get_timer_registers(this->id)->CR1 = TIM_CR1_UIFREMAP | TIM_CR1_ARPE;
    get_timer_registers(this->id)->SR  = 0;

    NVIC_SetPriority(IRQn_Type::TIM6_DAC_IRQn, a_irq_priority);
    NVIC_EnableIRQ(IRQn_Type::TIM6_DAC_IRQn);
}

void Basic_timer::disable()
{
    NVIC_DisableIRQ(IRQn_Type::TIM6_DAC_IRQn);

    get_timer_registers(this->id)->CR1 = 0;

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

} // namespace peripherals
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace soc::m4::stm32l4::peripherals;

void rcc<Basic_timer>::enable(Basic_timer::Id a_id, bool a_enable_in_lp)
{
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos + static_cast<uint32_t>(a_id));

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos + static_cast<uint32_t>(a_id));
    }
}

void rcc<Basic_timer>::disable(Basic_timer::Id a_id)
{
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos + static_cast<uint32_t>(a_id));
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos + static_cast<uint32_t>(a_id));
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif