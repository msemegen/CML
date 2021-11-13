/*
 *   Name: ADC.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/ADC/ADC.hpp>

// soc
#include <soc/system_timer.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/delay.hpp>
#include <cml/utils/wait_until.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;
using namespace cml::utils;

ADC::~ADC()
{
    if (true == bit_flag::is(this->p_registers->CR, ADC_CR_ADEN) &&
        false == bit_flag::is(this->p_registers->CR, ADC_CR_ADDIS))
    {
        this->disable();
    }
}

void ADC::enable(Resolution a_resolution)
{
    bit_flag::clear(&(this->p_registers->CR), ADC_CR_DEEPPWD);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADVREGEN);
    delay::us(21u);

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_ADCALDIF);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADCAL);

    wait_until::all_bits(&(this->p_registers->CR), ADC_CR_ADCAL, true);

    bit_flag::set(&(this->p_registers->CFGR), ADC_CFGR_RES_Msk, static_cast<std::uint32_t>(a_resolution));
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADEN);

    wait_until::all_bits(&(this->p_registers->ISR), ADC_ISR_ADRDY, false);

    bit_flag::set(&(this->p_registers->ISR), ADC_ISR_ADRDY);
}

bool ADC::enable(Resolution a_resolution, std::uint32_t a_timeout)
{
    std::uint32_t start = system_timer::get();

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_DEEPPWD);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADVREGEN);
    delay::us(21u);

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_ADCALDIF);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADCAL);

    bool ret = wait_until::all_bits(
        &(this->p_registers->CR), ADC_CR_ADCAL, true, start, a_timeout - (system_timer::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->CFGR), ADC_CFGR_RES_Msk, static_cast<std::uint32_t>(a_resolution));
        bit_flag::set(&(this->p_registers->CR), ADC_CR_ADEN);

        ret = wait_until::all_bits(
            &(this->p_registers->ISR), ADC_ISR_ADRDY, false, start, a_timeout - (system_timer::get() - start));
    }

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->ISR), ADC_ISR_ADRDY);
    }

    return ret;
}

void ADC::disable()
{
    cml_assert(false == bit_flag::is(p_registers->CR, ADC_CR_ADSTART) &&
               false == bit_flag::is(p_registers->CR, ADC_CR_JADSTART));

    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits(&(this->p_registers->CR), ADC_CR_ADDIS, true);

    bit_flag::set(&(this->p_registers->CR), ADC_CR_DEEPPWD);
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif