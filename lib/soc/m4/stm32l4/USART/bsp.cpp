/*
 *   Name: bsp.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/USART/bsp.hpp>

// std
#include <type_traits>

// cml
#include <cml/debug/assertion.hpp>
#include <cml/various.hpp>

namespace {
using namespace cml;
using namespace soc::m4::stm32l4;

struct Transfer_interrupt_context
{
    enum class Mode : std::uint32_t
    {
        USART,
        RS485
    };

    union
    {
        void* p_general = nullptr;
        USART* p_USART;
        RS485* p_RS485;
    };

    Mode mode = various::get_enum_incorrect_value<Mode>();
};

#if defined(SOC_USART1_PRESENT) && defined(SOC_USART2_PRESENT) && defined(SOC_USART3_PRESENT)
Transfer_interrupt_context transfer_irq_context[3];
#endif
#if defined(SOC_USART1_PRESENT) && defined(SOC_USART2_PRESENT) && !defined(SOC_USART3_PRESENT)
Transfer_interrupt_context transfer_irq_context[2];
#endif
} // namespace

extern "C" {
using namespace soc::m4::stm32l4;

#if defined(SOC_USART1_PRESENT)
void USART1_IRQHandler()
{
    cml_assert(nullptr != transfer_irq_context[0].p_general);
    cml_assert(various::get_enum_incorrect_value<Transfer_interrupt_context::Mode>() != transfer_irq_context[0].mode);

    switch (transfer_irq_context[0].mode)
    {
        case Transfer_interrupt_context::Mode::RS485: {
            RS485_interrupt_handler(transfer_irq_context[0].p_RS485);
        }
        break;

        case Transfer_interrupt_context::Mode::USART: {
            USART_interrupt_handler(transfer_irq_context[0].p_USART);
        }
        break;
    }
}
#endif

#if defined(SOC_USART2_PRESENT)
void USART2_IRQHandler()
{
    cml_assert(nullptr != transfer_irq_context[1].p_general);
    cml_assert(various::get_enum_incorrect_value<Transfer_interrupt_context::Mode>() != transfer_irq_context[1].mode);

    switch (transfer_irq_context[1].mode)
    {
        case Transfer_interrupt_context::Mode::RS485: {
            RS485_interrupt_handler(transfer_irq_context[1].p_RS485);
        }
        break;

        case Transfer_interrupt_context::Mode::USART: {
            USART_interrupt_handler(transfer_irq_context[1].p_USART);
        }
        break;
    }
}
#endif

#if defined(SOC_USART3_PRESENT)
void USART3_IRQHandler()
{
    cml_assert(nullptr != transfer_irq_context[2].p_general);
    cml_assert(various::get_enum_incorrect_value<Transfer_interrupt_context::Mode>() != transfer_irq_context[2].mode);

    switch (transfer_irq_context[2].mode)
    {
        case Transfer_interrupt_context::Mode::RS485: {
            RS485_interrupt_handler(transfer_irq_context[2].p_RS485);
        }
        break;

        case Transfer_interrupt_context::Mode::USART: {
            USART_interrupt_handler(transfer_irq_context[2].p_USART);
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

void USART::Interrupt::set_irq_context()
{
    cml_assert(nullptr == transfer_irq_context[this->p_USART->idx].p_general);

    transfer_irq_context[this->p_USART->idx] = { this->p_USART, Transfer_interrupt_context::Mode::USART };
}

void USART::Interrupt::clear_irq_context()
{
    cml_assert(nullptr != transfer_irq_context[this->p_USART->idx].p_general);

    transfer_irq_context[this->p_USART->idx] = {
        nullptr, various::get_enum_incorrect_value<Transfer_interrupt_context::Mode>()
    };
}

void RS485::Interrupt::set_irq_context()
{
    cml_assert(nullptr == transfer_irq_context[this->p_RS485->get_idx()].p_general);

    transfer_irq_context[this->p_RS485->get_idx()] = { this->p_RS485, Transfer_interrupt_context::Mode::RS485 };
}

void RS485::Interrupt::clear_irq_context()
{
    cml_assert(nullptr != transfer_irq_context[this->p_RS485->get_idx()].p_general);

    transfer_irq_context[this->p_RS485->get_idx()] = {
        nullptr, various::get_enum_incorrect_value<Transfer_interrupt_context::Mode>()
    };
}

#if defined(SOC_USART1_PRESENT)
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
#endif

#if defined(SOC_USART2_PRESENT)
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
#endif

#if defined(SOC_USART3_PRESENT)
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