/*
 *   Name: RNG.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/RNG/RNG.hpp>

// soc
#include <soc/m4/stm32l4/mcu/mcu.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace {

bool created = false;
#define RNG_T ((RNG_TypeDef*)RNG_BASE)

} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;
using namespace cml::utils;

RNG::RNG()
{
    cml_assert(false == created);
    created = true;
}

RNG::~RNG()
{
    this->disable();
    created = false;
}

bool RNG::enable(std::uint32_t a_timeout)
{
    cml_assert(rcc<mcu>::get_CLK48_frequency_Hz() <= 48_MHz);
    cml_assert(a_timeout > 0u);

    uint32_t start = system_timer::get();

    bit_flag::set(&(RNG_T->CR), RNG_CR_RNGEN);

    bool ret = false == bit_flag::is(RNG_T->SR, RNG_SR_SEIS);

    if (true == ret)
    {
        ret = wait_until::all_bits(&(RNG_T->SR), RNG_SR_SECS, true, start, a_timeout) &&
              wait_until::all_bits(&(RNG_T->SR), RNG_SR_CECS, true, start, a_timeout);
    }

    return ret;
}

void RNG::disable()
{
    bit_flag::clear(&(RNG_T->CR), RNG_CR_RNGEN);
    NVIC_DisableIRQ(RNG_IRQn);
}

void rcc<RNG>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_RNGEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_RNGSMEN);
    }
}
void rcc<RNG>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_RNGEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_RNGSMEN);
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4