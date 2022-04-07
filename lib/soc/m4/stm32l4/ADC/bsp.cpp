/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/ADC/bsp.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {
using namespace soc::m4::stm32l4;

ADC* irq_context[] = { nullptr,
#if defined(SOC_ADC2_PRESENT)
                       nullptr
#endif
};
} // namespace

extern "C" {
using namespace soc::m4::stm32l4;

#if defined(SOC_ADC2_PRESENT)
void ADC1_2_IRQHandler()
{
    cml_assert(nullptr != irq_context[0] || nullptr != irq_context[1]);

    if (nullptr != irq_context[0])
    {
        ADC_interrupt_handler(irq_context[0]);
    }

    if (nullptr != irq_context[1])
    {
        ADC_interrupt_handler(irq_context[1]);
    }
}
#endif
#if defined(SOC_ADC1_PRESENT) && !defined(SOC_ADC2_PRESENT)
void ADC1_IRQHandler()
{
    cml_assert(nullptr != irq_context[0]);
    ADC_interrupt_handler(irq_context[0]);
}
#endif
}

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void ADC::Interrupt::set_irq_context()
{
    cml_assert(nullptr == irq_context[this->p_ADC->idx]);

    irq_context[this->p_ADC->idx] = this->p_ADC;
}

void ADC::Interrupt::clear_irq_context()
{
    cml_assert(nullptr != irq_context[this->p_ADC->idx]);

    irq_context[this->p_ADC->idx] = nullptr;
}

template<>
void rcc<ADC>::enable<rcc<ADC>::Clock_source::PCLK>(rcc<ADC>::PCLK_prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    bit_flag::set(&(ADC_COMMON_T->CCR), ADC_CCR_CKMODE_Msk, static_cast<std::uint32_t>(a_prescaler));

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_ADCSMEN);
    }
}

#if defined(SOC_PLLSAI_PRESENT)
template<>
void rcc<ADC>::enable<rcc<ADC>::Clock_source::PLLSAI1>(rcc<ADC>::PLLSAI1_prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE_Msk);
    bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_PRESC_Msk, static_cast<std::uint32_t>(a_prescaler) << 18ul);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_ADCSMEN);
    }
}
#endif

void rcc<ADC>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_ADCSMEN);
}
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif