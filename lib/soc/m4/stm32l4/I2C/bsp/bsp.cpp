/*
 *   Name: bsp.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/m4/stm32l4/I2C/bsp/bsp.hpp>

// std
#include <type_traits>

// soc
#include <soc/m4/stm32l4/I2C/Interrupt.hpp>

// cml
#include <cml/debug/assertion.hpp>

#ifdef STM32L4

namespace {
using namespace cml;
using namespace soc::m4::stm32l4;

#if defined(STM32L412K8) || defined(STM32L412KB) || defined(STM32L412TB) || defined(STM32L422KB) || \
    defined(STM32L422TB) || defined(STM32L431KB) || defined(STM32L431KC) || defined(STM32L432KB) || \
    defined(STM32L432KC) || defined(STM32L442KC)
Interrupt<I2C>* irq_context[2] = { nullptr, nullptr };
#endif

#if defined(STM32L412R8) || defined(STM32L412T8) || defined(STM32L431CB) || defined(STM32L412C8) || \
    defined(STM32L412CB) || defined(STM32L412RB) || defined(STM32L422CB) || defined(STM32L422RB) || \
    defined(STM32L431CC) || defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC)
Interrupt<I2C>* irq_context[3] = { nullptr, nullptr, nullptr };
#endif

#if defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || \
    defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || \
    defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || \
    defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
Interrupt<I2C>* irq_context[4] = { nullptr, nullptr, nullptr, nullptr };
#endif
} // namespace

extern "C" {
using namespace soc::m4::stm32l4;

static void interrupt_handler(std::size_t a_index)
{
    cml_assert(nullptr != irq_context[a_index]);

    I2C_interrupt_handler(irq_context[a_index]);
}

void I2C1_EV_IRQHandler()
{
    interrupt_handler(0);
}
void I2C1_ER_IRQHandler()
{
    interrupt_handler(0);
}
#if defined(STM32L412R8) || defined(STM32L431CB) || defined(STM32L412C8) || defined(STM32L412CB) || \
    defined(STM32L412RB) || defined(STM32L422CB) || defined(STM32L422RB) || defined(STM32L431CC) || \
    defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L433CB) || \
    defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || defined(STM32L433VC) || \
    defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || defined(STM32L451CC) || \
    defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || defined(STM32L451VC) || \
    defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || defined(STM32L452RC) || \
    defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || defined(STM32L462CE) || \
    defined(STM32L462RE) || defined(STM32L462VE)

void I2C2_EV_IRQHandler()
{
    interrupt_handler(1);
}
void I2C2_ER_IRQHandler()
{
    interrupt_handler(1);
}
#endif

#if defined(STM32L412R8) || defined(STM32L412T8) || defined(STM32L431CB) || defined(STM32L412C8) || \
    defined(STM32L412CB) || defined(STM32L412RB) || defined(STM32L422CB) || defined(STM32L422RB) || \
    defined(STM32L431CC) || defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || \
    defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || \
    defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || \
    defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || \
    defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
void I2C3_EV_IRQHandler()
{
    interrupt_handler(2);
}
void I2C3_ER_IRQHandler()
{
    interrupt_handler(2);
}
#endif

#if defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || \
    defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || \
    defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || \
    defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
void I2C4_EV_IRQHandler()
{
    interrupt_handler(3);
}
void I2C4_ER_IRQHandler()
{
    interrupt_handler(3);
}
#endif
}

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

void Interrupt<I2C_master>::set_irq_context()
{
    cml_assert(nullptr == irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = this;
}

void Interrupt<I2C_master>::clear_irq_context()
{
    cml_assert(nullptr != irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = nullptr;
}

void Interrupt<I2C_slave>::set_irq_context()
{
    cml_assert(nullptr == irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = this;
}

void Interrupt<I2C_slave>::clear_irq_context()
{
    cml_assert(nullptr != irq_context[this->get_handle()->get_idx()]);

    irq_context[this->get_handle()->get_idx()] = nullptr;
}

template<> template<> void rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::HSI>(bool a_lp_enable)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_I2C1SEL, RCC_CCIPR_I2C1SEL_1);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN);
    }
}
template<> template<> void rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::PCLK1>(bool a_lp_enable)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_I2C1SEL);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN);
    }
}
template<> template<> void rcc<I2C, 1>::enable<rcc<I2C, 1>::Clock_source::SYSCLK>(bool a_lp_enable)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_I2C1SEL, RCC_CCIPR_I2C1SEL_0);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN);
    }
}
template<> void rcc<I2C, 1>::disable()
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN);
    bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN);
}
#if defined(STM32L412R8) || defined(STM32L431CB) || defined(STM32L412C8) || defined(STM32L412CB) || \
    defined(STM32L412RB) || defined(STM32L422CB) || defined(STM32L422RB) || defined(STM32L431CC) || \
    defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L433CB) || \
    defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || defined(STM32L433VC) || \
    defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || defined(STM32L451CC) || \
    defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || defined(STM32L451VC) || \
    defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || defined(STM32L452RC) || \
    defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || defined(STM32L462CE) || \
    defined(STM32L462RE) || defined(STM32L462VE)

template<> template<> void rcc<I2C, 2>::enable<rcc<I2C, 2>::Clock_source::HSI>(bool a_lp_enable)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_I2C2SEL, RCC_CCIPR_I2C2SEL_1);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C2EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C2SMEN);
    }
}
template<> template<> void rcc<I2C, 2>::enable<rcc<I2C, 2>::Clock_source::PCLK1>(bool a_lp_enable)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_I2C2SEL);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C2EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C2SMEN);
    }
}
template<> template<> void rcc<I2C, 2>::enable<rcc<I2C, 2>::Clock_source::SYSCLK>(bool a_lp_enable)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_I2C1SEL, RCC_CCIPR_I2C2SEL_0);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C2EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C2SMEN);
    }
}
template<> void rcc<I2C, 2>::disable()
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C2EN);
    bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C2SMEN);
}
#endif
#if defined(STM32L412R8) || defined(STM32L412T8) || defined(STM32L431CB) || defined(STM32L412C8) || \
    defined(STM32L412CB) || defined(STM32L412RB) || defined(STM32L422CB) || defined(STM32L422RB) || \
    defined(STM32L431CC) || defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || \
    defined(STM32L433CB) || defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || \
    defined(STM32L433VC) || defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || \
    defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || \
    defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || \
    defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || \
    defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
template<> template<> void rcc<I2C, 3>::enable<rcc<I2C, 3>::Clock_source::HSI>(bool a_lp_enable)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_I2C3SEL, RCC_CCIPR_I2C3SEL_1);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C3EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C3SMEN);
    }
}
template<> template<> void rcc<I2C, 3>::enable<rcc<I2C, 3>::Clock_source::PCLK1>(bool a_lp_enable)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_I2C3SEL);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C3EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C3SMEN);
    }
}
template<> template<> void rcc<I2C, 3>::enable<rcc<I2C, 3>::Clock_source::SYSCLK>(bool a_lp_enable)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_I2C3SEL, RCC_CCIPR_I2C3SEL_0);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C3EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C3SMEN);
    }
}
template<> void rcc<I2C, 3>::disable()
{
    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C3EN);
    bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C3SMEN);
}
#endif
#if defined(STM32L451CC) || defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || \
    defined(STM32L451VC) || defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || \
    defined(STM32L452RC) || defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || \
    defined(STM32L462CE) || defined(STM32L462RE) || defined(STM32L462VE)
template<> template<> void rcc<I2C, 4>::enable<rcc<I2C, 4>::Clock_source::HSI>(bool a_lp_enable)
{
    bit_flag::set(&(RCC->CCIPR2), RCC_CCIPR2_I2C4SEL, RCC_CCIPR2_I2C4SEL_1);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_I2C4SMEN);
    }
}
template<> template<> void rcc<I2C, 4>::enable<rcc<I2C, 4>::Clock_source::PCLK1>(bool a_lp_enable)
{
    bit_flag::clear(&(RCC->CCIPR2), RCC_CCIPR2_I2C4SEL);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_I2C4SMEN);
    }
}
template<> template<> void rcc<I2C, 4>::enable<rcc<I2C, 4>::Clock_source::SYSCLK>(bool a_lp_enable)
{
    bit_flag::set(&(RCC->CCIPR2), RCC_CCIPR2_I2C4SEL, RCC_CCIPR2_I2C4SEL_0);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN);

    if (true == a_lp_enable)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_I2C4SMEN);
    }
}
template<> void rcc<I2C, 4>::disable()
{
    bit_flag::clear(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN);
    bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_I2C4SMEN);
}
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif