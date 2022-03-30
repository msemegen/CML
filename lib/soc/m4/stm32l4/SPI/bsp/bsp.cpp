/*
 *   Name: bsp.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/SPI/bsp/bsp.hpp>

// soc
#include <soc/m4/stm32l4/SPI/Interrupt.hpp>

// cml
#include <cml/debug/assertion.hpp>
#include <cml/various.hpp>

#ifdef STM32L4

namespace {
using namespace cml;
using namespace soc::m4::stm32l4;

#if defined(SOC_SPI1_PRESENT) && !defined(SOC_SPI2_PRESENT) && !defined(SOC_SPI3_PRESENT)
Interrupt<SPI>* irq_context[1];
#endif

#if defined(SOC_SPI1_PRESENT) && ((defined(SOC_SPI2_PRESENT) && !defined(SOC_SPI3_PRESENT)) || \
                                  (!defined(SOC_SPI2_PRESENT) && defined(SOC_SPI3_PRESENT)))
Interrupt<SPI>* irq_context[2];
#endif

#if defined(SOC_SPI1_PRESENT) && defined(SOC_SPI2_PRESENT) && defined(SOC_SPI3_PRESENT)
Interrupt<SPI>* irq_context[3];
#endif
} // namespace

extern "C" {
using namespace soc::m4::stm32l4;

#if defined(SOC_SPI1_PRESENT)
void SPI1_IRQHandler()
{
    cml_assert(nullptr != irq_context[0]);

    SPI_interrupt_handler(&(irq_context[0]->transmission.rx));
    SPI_interrupt_handler(&(irq_context[0]->transmission.tx));
    SPI_interrupt_handler(&(irq_context[0]->status));
}
#endif

#if defined(SOC_SPI2_PRESENT)
void SPI2_IRQHandler()
{
    cml_assert(nullptr != irq_context[1]);

    SPI_interrupt_handler(&(irq_context[1]->transmission.rx));
    SPI_interrupt_handler(&(irq_context[1]->transmission.tx));
    SPI_interrupt_handler(&(irq_context[1]->status));
}
#endif

#if defined(SOC_SPI3_PRESENT) 
void SPI3_IRQHandler()
{
    cml_assert(nullptr != irq_context[2]);

    SPI_interrupt_handler(&(irq_context[2]->transmission.rx));
    SPI_interrupt_handler(&(irq_context[2]->transmission.tx));
    SPI_interrupt_handler(&(irq_context[2]->status));
}
#endif
}
namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void Interrupt<SPI_master>::set_irq_context()
{
    cml_assert(nullptr == irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = this;
}

void Interrupt<SPI_master>::clear_irq_context()
{
    cml_assert(nullptr != irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = nullptr;
}

void Interrupt<SPI_slave>::set_irq_context()
{
    cml_assert(nullptr == irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = this;
}

void Interrupt<SPI_slave>::clear_irq_context()
{
    cml_assert(nullptr != irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = nullptr;
}

#if defined(SOC_SPI1_PRESENT)
template<> void rcc<SPI, 1>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN);
    }
}
template<> void rcc<SPI, 1>::disable()
{
    bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);
    bit_flag::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN);
}
#endif

#if defined(SOC_SPI2_PRESENT) 
template <> void rcc <SPI, 2u>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI2SMEN);
    }
}
template<> void rcc<SPI, 2u>::disable()
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN);
    bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI2SMEN);
}
#endif

#if defined(SOC_SPI3_PRESENT)
template<> void rcc<SPI, 3u>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI3EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI3SMEN);
    }
}
template<> void rcc<SPI, 3u>::disable()
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI3EN);
    bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI3SMEN);
}
#endif

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif