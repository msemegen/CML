/*
 *   Name: bsp.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/ADC/bsp/bsp.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {
using namespace soc::m4::stm32l4;

Interrupt<ADC>* irq_context[] = { nullptr,
#if defined(STM32L412xx) || defined(STM32L422xx)
                                  nullptr
#endif
};
} // namespace

extern "C" {

#if defined(STM32L412xx) || defined(STM32L422xx)
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
#else
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

void Interrupt<ADC>::set_irq_context()
{
    cml_assert(nullptr == irq_context[this->p_adc->get_idx()]);

    irq_context[this->p_adc->get_idx()] = this;
}

void Interrupt<ADC>::clear_irq_context()
{
    cml_assert(nullptr != irq_context[this->p_adc->get_idx()]);

    irq_context[this->p_adc->get_idx()] = nullptr;
}

template<>
void rcc<ADC>::enable<rcc<ADC>::Clock_source::PCLK>(rcc<ADC>::PCLK_prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);

#if defined(STM32L412xx) || defined(STM32L422xx)
    bit_flag::set(&(ADC12_COMMON->CCR), ADC_CCR_CKMODE_Msk, static_cast<std::uint32_t>(a_prescaler) << 16ul);
#else
    bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE_Msk, static_cast<std::uint32_t>(a_prescaler));
#endif

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_ADCSMEN);
    }
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
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