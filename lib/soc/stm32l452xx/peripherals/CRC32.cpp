/*
 *   Name: CRC32.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/peripherals/CRC32.hpp>

// cml
#include <cml/debug/assertion.hpp>

namespace {
bool created = false;
} // namespace

namespace soc {
namespace stm32l452xx {
namespace peripherals {

using namespace cml;

CRC32::CRC32()
{
    cml_assert(false == created);
    created = true;
}

CRC32::~CRC32()
{
    created = false;
}

void CRC32::enable(In_data_reverse a_in_reverse, Out_data_reverse a_out_reverse)
{
    bit_flag::set(&(RCC->AHB1ENR), RCC_AHB1ENR_CRCEN);

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

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx