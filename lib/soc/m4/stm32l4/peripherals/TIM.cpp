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

namespace {

using namespace cml;
using namespace soc::m4::stm32l4::peripherals;

struct Basic_timer_controller
{
    TIM_TypeDef* p_registers       = nullptr;
    TIM::Basic::_6* p_timer_handle = nullptr;
};

struct General_purpose_timer_controller
{
    TIM_TypeDef* p_registers = nullptr;
    union
    {
        void* p_general_handle = nullptr;
        TIM::General_purpose::_2* p_timer2_handle;
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        TIM::General_purpose::_3* p_timer3_handle;
#endif
        TIM::General_purpose::_15* p_timer15_handle;
        TIM::General_purpose::_16* p_timer16_handle;
    };
};

Basic_timer_controller basic_timer_controllers[] = { { TIM6, nullptr },
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
                                                     { TIM7, nullptr }
#endif
};

General_purpose_timer_controller general_purpose_timer_controllers[] = {
    { TIM2, nullptr },
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    { TIM3, nullptr },
#endif
    { TIM15, nullptr },
    { TIM16, nullptr },
};

TIM_TypeDef* get_timer_registers(TIM::Basic::Id a_id)
{
    return basic_timer_controllers[static_cast<uint32_t>(a_id)].p_registers;
}

TIM_TypeDef* get_timer_registers(TIM::General_purpose::Id a_id)
{
    return general_purpose_timer_controllers[static_cast<uint32_t>(a_id)].p_registers;
}

} // namespace

extern "C" {

void TIM6_DAC_IRQHandler()
{
    basic_timer6_interrupt_handler(basic_timer_controllers[0].p_timer_handle);
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

void basic_timer6_interrupt_handler(TIM::Basic::_6* a_p_this)
{
    cml_assert(nullptr != a_p_this);
    cml_assert(nullptr != a_p_this->overload_callback.function);

    a_p_this->overload_callback.function(a_p_this->overload_callback.p_user_data);

    TIM6->SR = 0;
}

void interrupt_handler(TIM::General_purpose* a_p_this)
{
    cml_assert(nullptr != a_p_this);
}

void TIM::Basic::_6::set_IRQ(const TIM::IRQ& a_config)
{
    cml_assert(various::get_enum_incorrect_value<IRQ::Mode>() != a_config.mode);

    switch (a_config.mode)
    {
        case IRQ::Mode::enabled: {
            NVIC_SetPriority(
                IRQn_Type::TIM6_DAC_IRQn,
                NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority));
            NVIC_EnableIRQ(IRQn_Type::TIM6_DAC_IRQn);
        }
        break;

        case IRQ::Mode::disabled: {
            NVIC_DisableIRQ(IRQn_Type::TIM6_DAC_IRQn);
        }
        break;
    }
}

void TIM::Basic::_6::set_time_base(const Time_base& a_config)
{
    cml_assert(various::get_enum_incorrect_value<Time_base::Mode>() != a_config.mode);
    cml_assert(various::get_enum_incorrect_value<Time_base::Autoreload_preload>() != a_config.arr_preload);
    cml_assert(
        (Time_base::Mode::enabled == a_config.mode && a_config.arr_preload != Time_base::Autoreload_preload::none) ||
        (Time_base::Mode::disabled == a_config.mode && a_config.arr_preload == Time_base::Autoreload_preload::none));

    switch (a_config.mode)
    {
        case Time_base::Mode::enabled: {
            TIM6->PSC = a_config.prescaler;
            TIM6->CR1 = static_cast<uint32_t>(a_config.arr_preload);
            TIM6->SR  = 0;
        }
        break;

        case Time_base::Mode::disabled: {
            TIM6->CR1 = 0;
        }
        break;
    }
}

void TIM::Basic::_6::start(uint16_t a_auto_reload)
{
    cml_assert(0 != a_auto_reload);

    TIM6->ARR = a_auto_reload;
    bit_flag::set(&(TIM6->CR1), TIM_CR1_CEN);
}

void TIM::Basic::_6::stop()
{
    bit_flag::clear(&(TIM6->CR1), TIM_CR1_CEN);
}

void TIM::Basic::_6::register_overload_callback(const Overload_callback& a_callback)
{
    Interrupt_guard guard;

    this->overload_callback = a_callback;
    TIM6->DIER              = TIM_DIER_UIE;
}

void TIM::Basic::_6::unregister_overload_callback()
{
    Interrupt_guard guard;

    this->overload_callback = { nullptr, nullptr };
    TIM6->DIER              = 0;
}

bool TIM::Basic::_6::is_overload_event() const
{
    cml_assert(nullptr == this->overload_callback.function);

    if (true == bit_flag::is(TIM6->SR, TIM_SR_UIF))
    {
        TIM6->SR = 0;
        return true;
    }

    return false;
}

TIM::IRQ TIM::Basic::_6::get_IRQ() const
{
    IRQ ret;

    ret.mode = static_cast<IRQ::Mode>(NVIC_GetEnableIRQ(IRQn_Type::TIM6_DAC_IRQn));
    NVIC_DecodePriority(NVIC_GetPriority(IRQn_Type::TIM6_DAC_IRQn),
                        NVIC_GetPriorityGrouping(),
                        &(ret.preempt_priority),
                        &(ret.sub_priority));

    return ret;
}

void TIM::General_purpose::_2::set_IRQ(const IRQ& a_config) {}

void TIM::General_purpose::_2::set_time_base(const Time_base& a_config)
{
    cml_assert(various::get_enum_incorrect_value<Time_base::Mode>() != a_config.mode);
    cml_assert(various::get_enum_incorrect_value<Time_base::Autoreload_preload>() != a_config.arr_preload);
    cml_assert(various::get_enum_incorrect_value<Time_base::Direction>() != a_config.direction);
    cml_assert(
        (Time_base::Mode::enabled == a_config.mode && a_config.arr_preload != Time_base::Autoreload_preload::none &&
         Time_base::Mode::enabled == a_config.mode && a_config.direction != Time_base::Direction::none) ||
        (Time_base::Mode::disabled == a_config.mode && a_config.arr_preload == Time_base::Autoreload_preload::none &&
         Time_base::Mode::disabled == a_config.mode && a_config.direction == Time_base::Direction::none));

    switch (a_config.mode)
    {
        case Time_base::Mode::enabled: {
            TIM2->PSC = a_config.prescaler;
            TIM2->CR1 = static_cast<uint32_t>(a_config.arr_preload);
            TIM2->SR  = 0;
        }
        break;

        case Time_base::Mode::disabled: {
            TIM2->CR1 = 0;
        }
        break;
    }
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