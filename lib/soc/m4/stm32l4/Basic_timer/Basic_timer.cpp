/*
 *   Name: Basic_timer.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/Basic_timer/Basic_timer.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;

void Basic_timer::enable(const Enable_config& a_config)
{
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
    this->p_registers->CR1 = 0x0u;
}

void Basic_timer::start()
{
    bit_flag::set(&(this->p_registers->CR1), TIM_CR1_CEN);
}

void Basic_timer::stop()
{
    bit_flag::clear(&(this->p_registers->CR1), TIM_CR1_CEN);
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif