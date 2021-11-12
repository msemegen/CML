/*
 *   Name: bsp.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/USART/bsp/bsp.hpp>

// std
#include <type_traits>

// cml
#include <cml/debug/assertion.hpp>
#include <cml/various.hpp>

namespace {
using namespace cml;
using namespace soc::m4::stm32l4;

struct Interrupt_context
{
    enum class Mode : std::uint32_t
    {
        USART,
        RS485
    };

    union
    {
        void* p_general = nullptr;
        Interrupt<USART>* p_USART;
        Interrupt<RS485>* p_RS485;
    };

    Mode mode = various::get_enum_incorrect_value<Mode>();
};

#if defined(STM32L412R8) || defined(STM32L431CB) || defined(STM32L412C8) || defined(STM32L412CB) || \
    defined(STM32L412RB) || defined(STM32L422CB) || defined(STM32L422RB) || defined(STM32L431CC) || \
    defined(STM32L431RB) || defined(STM32L431RC) || defined(STM32L431VC) || defined(STM32L433CB) || \
    defined(STM32L433CC) || defined(STM32L433RB) || defined(STM32L433RC) || defined(STM32L433VC) || \
    defined(STM32L443CC) || defined(STM32L443RC) || defined(STM32L443VC) || defined(STM32L451CC) || \
    defined(STM32L451CE) || defined(STM32L451RC) || defined(STM32L451RE) || defined(STM32L451VC) || \
    defined(STM32L451VE) || defined(STM32L452CC) || defined(STM32L452CE) || defined(STM32L452RC) || \
    defined(STM32L452RE) || defined(STM32L452VC) || defined(STM32L452VE) || defined(STM32L462CE) || \
    defined(STM32L462RE) || defined(STM32L462VE)
Interrupt_context irq_context[3];
#endif
#if defined(STM32L412K8) || defined(STM32L412KB) || defined(STM32L412TB) || defined(STM32L422KB) || \
    defined(STM32L422TB) || defined(STM32L431KB) || defined(STM32L431KC) || defined(STM32L432KB) || \
    defined(STM32L432KC) || defined(STM32L442KC) || defined(STM32L412T8)
Context irq_context[2];
#endif
} // namespace

extern "C" {
using namespace soc::m4::stm32l4;

void USART1_IRQHandler()
{
    cml_assert(nullptr != irq_context[0].p_general);
    cml_assert(various::get_enum_incorrect_value<Interrupt_context::Mode>() != irq_context[0].mode);

    switch (irq_context[0].mode)
    {
        case Interrupt_context::Mode::RS485: {
            RS485_interrupt_handler(irq_context[0].p_RS485);
        }
        break;

        case Interrupt_context::Mode::USART: {
            USART_interrupt_handler(irq_context[0].p_USART);
        }
        break;
    }
}

void USART2_IRQHandler()
{
    cml_assert(nullptr != irq_context[1].p_general);
    cml_assert(various::get_enum_incorrect_value<Interrupt_context::Mode>() != irq_context[1].mode);

    switch (irq_context[1].mode)
    {
        case Interrupt_context::Mode::RS485: {
            RS485_interrupt_handler(irq_context[1].p_RS485);
        }
        break;

        case Interrupt_context::Mode::USART: {
            USART_interrupt_handler(irq_context[1].p_USART);
        }
        break;
    }
}

#if defined(STM32L412R8) || defined(STM32L412C8) || defined(STM32L412CB) || defined(STM32L412RB)
void USART3_IRQHandler()
{
    cml_assert(nullptr != irq_context[2].p_general);
    cml_assert(various::get_enum_incorrect_value<Interrupt_context::Mode>() != irq_context[2].mode);

    switch (irq_context[2].mode)
    {
        case Interrupt_context::Mode::RS485: {
            RS485_interrupt_handler(irq_context[2].p_RS485);
        }
        break;

        case Interrupt_context::Mode::USART: {
            USART_interrupt_handler(irq_context[2].p_USART);
        }
        break;
    }
}
#endif
}
namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;

Interrupt<USART>::Interrupt(USART* a_p_USART, IRQn_Type a_irqn)
    : transmission(*a_p_USART)
    , status(*a_p_USART)
    , p_USART(a_p_USART)
    , irqn(a_irqn)
{
    cml_assert(nullptr == irq_context[this->p_USART->get_idx()].p_general);

    irq_context[this->p_USART->get_idx()] = { this, Interrupt_context::Mode::USART };
}

Interrupt<USART>::~Interrupt()
{
    cml_assert(nullptr != irq_context[this->p_USART->get_idx()].p_general);

    this->disable();

    for (std::size_t i = 0; i < std::extent<decltype(irq_context)>::value; i++)
    {
        if (static_cast<Interrupt<USART>*>(this) == irq_context[i].p_USART)
        {
            irq_context[i].p_general = nullptr;
            break;
        }
    }
}

Interrupt<RS485>::Interrupt(RS485* a_p_RS485, IRQn_Type a_irqn)
    : transmission(*a_p_RS485)
    , status(*a_p_RS485)
    , p_RS485(a_p_RS485)
    , irqn(a_irqn)
{
    cml_assert(nullptr == irq_context[this->p_RS485->get_idx()].p_general);

    irq_context[this->p_RS485->get_idx()] = { this, Interrupt_context::Mode::USART };
}

Interrupt<RS485>::~Interrupt()
{
    cml_assert(nullptr != irq_context[this->p_RS485->get_idx()].p_general);

    this->disable();

    for (std::size_t i = 0; i < std::extent<decltype(irq_context)>::value; i++)
    {
        if (static_cast<Interrupt<RS485>*>(this) == irq_context[i].p_RS485)
        {
            irq_context[i].p_general = nullptr;
            break;
        }
    }
}

template<> template<> void rcc<USART, 1u>::enable<rcc<USART, 1u>::Clock_source::HSI>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART1SEL_0 | RCC_CCIPR_USART1SEL_1, RCC_CCIPR_USART1SEL_1);
    bit::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN_Pos);
    }
}
template<> template<> void rcc<USART, 1u>::enable<rcc<USART, 1u>::Clock_source::PCLK1>(bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART1SEL);
    bit::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN_Pos);
    }
}
template<> template<> void rcc<USART, 1u>::enable<rcc<USART, 1u>::Clock_source::SYSCLK>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART1SEL_0 | RCC_CCIPR_USART1SEL_1, RCC_CCIPR_USART1SEL_0);
    bit::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN_Pos);
    }
}
template<> void rcc<USART, 1u>::disable()
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART1SEL);
    bit::clear(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN_Pos);
    bit::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN_Pos);
}

template<> template<> void rcc<USART, 2u>::enable<rcc<USART, 2u>::Clock_source::HSI>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART2SEL_0 | RCC_CCIPR_USART2SEL_1, RCC_CCIPR_USART2SEL_1);
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART2SMEN_Pos);
    }
}
template<> template<> void rcc<USART, 2u>::enable<rcc<USART, 2u>::Clock_source::PCLK1>(bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART2SEL_0 | RCC_CCIPR_USART2SEL_1);
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART2SMEN_Pos);
    }
}
template<> template<> void rcc<USART, 2u>::enable<rcc<USART, 2u>::Clock_source::SYSCLK>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART2SEL_0 | RCC_CCIPR_USART2SEL_1, RCC_CCIPR_USART2SEL_0);
    bit::set(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART2SMEN_Pos);
    }
}
template<> void rcc<USART, 2u>::disable()
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART2SEL);
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_USART2EN_Pos);
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART2SMEN_Pos);
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
template<> template<> void rcc<USART, 3u>::enable<rcc<USART, 3u>::Clock_source::HSI>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART3SEL_0 | RCC_CCIPR_USART3SEL_1, RCC_CCIPR_USART3SEL_1);
    bit::set(&(RCC->APB2ENR), RCC_APB1ENR1_USART3EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART3SMEN_Pos);
    }
}
template<> template<> void rcc<USART, 3u>::enable<rcc<USART, 3u>::Clock_source::PCLK1>(bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART3SEL_0 | RCC_CCIPR_USART3SEL_1);
    bit::set(&(RCC->APB2ENR), RCC_APB1ENR1_USART3EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART3SMEN_Pos);
    }
}
template<> template<> void rcc<USART, 3u>::enable<rcc<USART, 3u>::Clock_source::SYSCLK>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART3SEL_0 | RCC_CCIPR_USART3SEL_1, RCC_CCIPR_USART3SEL_0);
    bit::set(&(RCC->APB2ENR), RCC_APB1ENR1_USART3EN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART3SMEN_Pos);
    }
}
template<> void rcc<USART, 3>::disable()
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART3SEL);
    bit::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_USART3EN_Pos);
    bit::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_USART3SMEN_Pos);
}
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif