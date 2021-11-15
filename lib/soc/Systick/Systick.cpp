/*
 *   Name: Systick.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/Systick/Systick.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
using namespace cml;

Systick::~Systick()
{
    if (true == this->is_enabled())
    {
        this->disable();
    }
}

void Systick::enable(std::uint32_t a_start_value, Prescaler a_prescaler)
{
    cml_assert(a_start_value > 0);
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    SysTick->CTRL = 0;
    SysTick->LOAD = a_start_value;
    SysTick->VAL  = 0;
    SysTick->CTRL = static_cast<std::uint32_t>(a_prescaler) | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void Systick::disable()
{
    cml_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    SysTick->CTRL = 0;
}

bool Systick::is_enabled()
{
    return bit_flag::is(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

} // namespace soc
