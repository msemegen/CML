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

#if defined(STM32L412T8) || defined(STM32L412K8) || defined(STM32L412KB) || defined(STM32L412TB) || \
    defined(STM32L422KB) || defined(STM32L422TB)
Interrupt<SPI>* irq_context[1];
#endif

#if defined(STM32L412R8) || defined(STM32L412C8) || defined(STM32L412CB) || defined(STM32L412RB) || \
    defined(STM32L422CB) || defined(STM32L422RB) || defined(STM32L431KB) || defined(STM32L431KC) || \
    defined(STM32L442KC) || defined(STM32L432KB) || defined(STM32L432KC)
Interrupt<SPI>* irq_context[2];
#endif

#if defined(STM32L431CB) || defined(STM32L431CC) || defined(STM32L431RB) || defined(STM32L431RC) || \
    defined(STM32L431VC) || defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || \
    defined(STM32L433RC) || defined(STM32L433VC) || defined(STM32L443CC) || defined(STM32L443RC) || \
    defined(STM32L443VC) || defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || \
    defined(STM32L451RE) || defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || \
    defined(STM32L452CE) || defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || \
    defined(STM32L452VE) || defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
Interrupt<SPI>* irq_context[3];
#endif
} // namespace

extern "C" {
using namespace soc::m4::stm32l4;
void SPI1_IRQHandler()
{
    cml_assert(nullptr != irq_context[0]);

    SPI_interrupt_handler(&(irq_context[0]->transmission.rx));
    SPI_interrupt_handler(&(irq_context[0]->transmission.tx));
    SPI_interrupt_handler(&(irq_context[0]->status));
}
#if defined(STM32L412R8) || defined(STM32L412C8) || defined(STM32L412CB) || defined(STM32L412RB) || \
    defined(STM32L422CB) || defined(STM32L422RB) || defined(STM32L431KB) || defined(STM32L431KC) || \
    defined(STM32L442KC) || defined(STM32L432KB) || defined(STM32L432KC) || defined(STM32L431CB) || \
    defined(STM32L431CC) || defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || \
    defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || \
    defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || \
    defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || \
    defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
void SPI2_IRQHandler()
{
    cml_assert(nullptr != irq_context[1]);

    SPI_interrupt_handler(&(irq_context[1]->transmission.rx));
    SPI_interrupt_handler(&(irq_context[1]->transmission.tx));
    SPI_interrupt_handler(&(irq_context[1]->status));
}
#endif
#if defined(STM32L431CB) || defined(STM32L431CC) || defined(STM32L431RB) || defined(STM32L431RC) || \
    defined(STM32L431VC) || defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || \
    defined(STM32L433RC) || defined(STM32L433VC) || defined(STM32L443CC) || defined(STM32L443RC) || \
    defined(STM32L443VC) || defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || \
    defined(STM32L451RE) || defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || \
    defined(STM32L452CE) || defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || \
    defined(STM32L452VE) || defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
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

Interrupt<SPI_master>::Interrupt(SPI_master* a_p_SPI, IRQn_Type a_irqn)
    : Interrupt<SPI>(*a_p_SPI, a_irqn)
    , p_SPI(a_p_SPI)
{
    cml_assert(nullptr == irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = this;
}

Interrupt<SPI_master>::~Interrupt()
{
    cml_assert(nullptr != irq_context[this->get_handle()->get_idx()]);

    for (std::size_t i = 0; i < std::extent<decltype(irq_context)>::value; i++)
    {
        if (static_cast<Interrupt<SPI>*>(this) == irq_context[i])
        {
            irq_context[i] = nullptr;
            break;
        }
    }
}

Interrupt<SPI_slave>::Interrupt(SPI_slave* a_p_SPI, IRQn_Type a_irqn)
    : Interrupt<SPI>(*a_p_SPI, a_irqn)
    , p_SPI(a_p_SPI)
{
    cml_assert(nullptr == irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = this;
}

Interrupt<SPI_slave>::~Interrupt()
{
    cml_assert(nullptr != irq_context[this->get_handle()->get_idx()]);

    for (std::size_t i = 0; i < std::extent<decltype(irq_context)>::value; i++)
    {
        if (static_cast<Interrupt<SPI>*>(this) == irq_context[i])
        {
            irq_context[i] = nullptr;
            break;
        }
    }
}

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

#if defined(STM32L412R8) || defined(STM32L412C8) || defined(STM32L412CB) || defined(STM32L412RB) || \
    defined(STM32L422CB) || defined(STM32L422RB) || defined(STM32L431KB) || defined(STM32L431KC) || \
    defined(STM32L442KC) || defined(STM32L432KB) || defined(STM32L432KC) || defined(STM32L431CB) || \
    defined(STM32L431CC) || defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || \
    defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || \
    defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || \
    defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || \
    defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
template<> void rcc<SPI, 2u>::enable(bool a_enable_in_lp)
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

#if defined(STM32L431CB) || defined(STM32L431CC) || defined(STM32L431RB) || defined(STM32L431RC) || \
    defined(STM32L431VC) || defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || \
    defined(STM32L433RC) || defined(STM32L433VC) || defined(STM32L443CC) || defined(STM32L443RC) || \
    defined(STM32L443VC) || defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || \
    defined(STM32L451RE) || defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || \
    defined(STM32L452CE) || defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || \
    defined(STM32L452VE) || defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
template<> void rcc<SPI, 3u>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI2SMEN);
    }
}
template<> void rcc<SPI, 3u>::disable()
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_SPI2EN);
    bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_SPI2SMEN);
}
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc
#endif