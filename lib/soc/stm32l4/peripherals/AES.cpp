/*
 *   Name: AES.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/stm32l4/peripherals/AES.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

#if defined(STM32L422xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L462xx)

namespace {

bool created = false;

} // namespace

namespace soc {
namespace stm32l4 {
namespace peripherals {

#define AES_T ((RNG_TypeDef*)AES_BASE)

AES::AES()
{
    cml_assert(false == created);
    created = false;
}

AES::~AES()
{
    this->disable();
    created = false;
}

void AES::enable(const Config& a_config)
{
    bit_flag::clear(&(AES_T->CR), AES_CR_EN);

    switch (a_config.chaining_mode)
    {
        case Config::Chaining_mode::ecb: {

        }
        break;

        case Config::Chaining_mode::cbc: {
        }
        break;

        case Config::Chaining_mode::ctr: {
        }
        break;

        case Config::Chaining_mode::gcm: {
        }
        break;

        case Config::Chaining_mode::gmac: {
        }
        break;

        case Config::Chaining_mode::ccm: {
        }
        break;
    }
}

void AES::disable()
{
    bit_flag::clear(&(AES_T->CR), AES_CR_EN);
}

} // namespace peripherals
} // namespace stm32l4
} // namespace soc

namespace soc {
namespace stm32l4 {

using namespace cml;
using namespace soc::stm32l4::peripherals;

void rcc<AES>::enable(bool a_enable_in_lp)
{
    bit::set(&(RCC->AHB2ENR), RCC_AHB2ENR_AESEN_Pos);

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_AESSMEN_Pos);
    }
}

void rcc<AES>::disable()
{
    bit::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_AESEN_Pos);
    bit::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_AESSMEN_Pos);
}

} // namespace stm32l4
} // namespace soc

#endif
#endif // STM32L4