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
#include <soc/Interrupt_guard.hpp>

namespace {

#define WWDG_T ((WWDG_TypeDef*)WWDG_BASE)

} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

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
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif