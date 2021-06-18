/*
 *   Name: CRC32.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/stm32l4/peripherals/CRC32.hpp>

// cml
#include <cml/debug/assertion.hpp>

namespace {
bool created = false;
} // namespace

namespace soc {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

using namespace cml;

CRC32::CRC32()
{
    cml_assert(false == created);
    created = true;
}

CRC32::~CRC32()
{
    this->disable();
    created = false;
}

void CRC32::enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse)
{
    bit_flag::clear(&(CRC->CR), CRC_CR_POLYSIZE);
    bit_flag::set(&(CRC->CR), static_cast<uint32_t>(a_in_reverse));
    bit_flag::set(&(CRC->CR), static_cast<uint32_t>(a_out_reverse));
}

void CRC32::disable()
{
    bit_flag::clear(&(RCC->AHB1ENR), RCC_AHB1ENR_CRCEN);
}

uint32_t CRC32::calculate(const uint8_t* a_p_data, uint32_t a_data_size)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_data_size > 0);

    bit_flag::set(&(CRC->CR), CRC_CR_RESET);

    for (uint32_t i = 0; i < a_data_size; i++)
    {
        CRC->DR = a_p_data[i];
    }

    return CRC->DR;
}

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc

namespace soc {
namespace stm32l4 {

using namespace soc::stm32l4::peripherals;

void rcc<CRC32>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB1ENR), RCC_AHB1ENR_CRCEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB1SMENR), RCC_AHB1SMENR_CRCSMEN);
    }
}

void rcc<CRC32>::disable()
{
    bit_flag::clear(&(RCC->AHB1ENR), RCC_AHB1ENR_CRCEN);
    bit_flag::clear(&(RCC->AHB1SMENR), RCC_AHB1SMENR_CRCSMEN);
}

} // namespace stm32l4
} // namespace soc

#endif // STM32L4