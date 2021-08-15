/*
 *   Name: TIM.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/peripherals/TIM.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// externals
#include <stm32l4xx.h>

namespace {

using namespace cml;
using namespace soc::m4::stm32l4::peripherals;

struct Controller
{
    TIM_TypeDef* p_registers = nullptr;
    union
    {
        TIM::Basic* p_basic_timer_handle = nullptr;
        TIM::General_puprose* p_general_purpose_timer_handle;
    };
};

Controller controllers[] = { { TIM6, nullptr },
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
                             { TIM7, nullptr }
#endif
};

TIM_TypeDef* get_timer_registers(TIM::Basic::Id a_id)
{
    return controllers[static_cast<uint32_t>(a_id)].p_registers;
}

} // namespace

extern "C" {

void TIM6_DAC_IRQHandler()
{
    interrupt_handler(controllers[0].p_basic_timer_handle);
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
void TIM7_IRQHandler()
{
    interrupt_handler(controllers[1].p_basic_timer_handle);
}
#endif
}

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

void interrupt_handler(TIM::Basic* a_p_this)
{
    cml_assert(nullptr != a_p_this);
    cml_assert(nullptr != a_p_this->overload_callback.function);

    a_p_this->overload_callback.function(a_p_this->overload_callback.p_user_data);

    get_timer_registers(a_p_this->id)->SR = 0;
}

void interrupt_handler(TIM::General_puprose* a_p_this)
{
    cml_assert(nullptr != a_p_this);
}

void TIM::Basic::enable_time_base(const Config& a_config, uint32_t a_irq_priority)
{
    controllers[static_cast<uint32_t>(this->id)].p_basic_timer_handle = this;

    get_timer_registers(this->id)->CNT = 0;
    get_timer_registers(this->id)->PSC = a_config.prescaler;
    get_timer_registers(this->id)->ARR = a_config.auto_reload;

    get_timer_registers(this->id)->CR1 = TIM_CR1_UIFREMAP | TIM_CR1_ARPE;
    get_timer_registers(this->id)->SR  = 0;

    NVIC_SetPriority(
        static_cast<IRQn_Type>(static_cast<uint32_t>(IRQn_Type::TIM6_DAC_IRQn) + static_cast<uint32_t>(this->id)),
        a_irq_priority);
    NVIC_EnableIRQ(
        static_cast<IRQn_Type>(static_cast<uint32_t>(IRQn_Type::TIM6_DAC_IRQn) + static_cast<uint32_t>(this->id)));
}

void TIM::Basic::disable_time_base()
{
    NVIC_DisableIRQ(
        static_cast<IRQn_Type>(static_cast<uint32_t>(IRQn_Type::TIM6_DAC_IRQn) + static_cast<uint32_t>(this->id)));

    get_timer_registers(this->id)->CR1 = 0;

    controllers[static_cast<uint32_t>(this->id)].p_basic_timer_handle = nullptr;
}

void TIM::Basic::start()
{
    bit_flag::set(&(get_timer_registers(this->id)->CR1), TIM_CR1_CEN);
}

void TIM::Basic::stop()
{
    bit_flag::clear(&(get_timer_registers(this->id)->CR1), TIM_CR1_CEN);
}

void TIM::Basic::register_overload_callback(const Overload_callback& a_callback)
{
    Interrupt_guard guard;

    this->overload_callback             = a_callback;
    get_timer_registers(this->id)->DIER = TIM_DIER_UIE;
}

void TIM::Basic::unregister_overload_callback()
{
    Interrupt_guard guard;

    this->overload_callback             = { nullptr, nullptr };
    get_timer_registers(this->id)->DIER = 0;
}

bool TIM::Basic::is_overload_event() const
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

using namespace cml;
using namespace soc::m4::stm32l4::peripherals;

void rcc<TIM::Basic>::enable(TIM::Basic::Id a_id, bool a_enable_in_lp)
{
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos + static_cast<uint32_t>(a_id));

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos + static_cast<uint32_t>(a_id));
    }
}

void rcc<TIM::Basic>::disable(TIM::Basic::Id a_id)
{
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_TIM6EN_Pos + static_cast<uint32_t>(a_id));
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_TIM6SMEN_Pos + static_cast<uint32_t>(a_id));
}

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif