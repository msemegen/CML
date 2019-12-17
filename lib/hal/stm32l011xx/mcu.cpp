/*
    Name: mcu.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

//this
#include <common/assert.hpp>
#include <common/bit.hpp>
#include <common/handy.hpp>
#include <hal/stm32l011xx/mcu.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

using namespace cml::common;

void c_mcu::enable_msi_clock(e_msi_frequency a_freq)
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    clear_flag(&(RCC->CR), RCC_ICSCR_MSIRANGE);
    set_flag(&(RCC->CR), static_cast<uint32>(a_freq) << RCC_ICSCR_MSIRANGE_Pos);

    set_flag(&(RCC->CR), RCC_CR_MSION);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));

    clear_flag(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));
    set_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::msi));
}

void c_mcu::enable_hsi_clock(e_hsi_frequency a_freq)
{
    _unused(a_freq);

    clear_flag(&(RCC->CR), RCC_CR_HSION);
    set_flag(&(RCC->CR), RCC_CR_HSION);

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY));
    set_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::hsi));
}

void c_mcu::enable_lsi_clock(e_lsi_frequency a_freq)
{
    _unused(a_freq);

    clear_flag(&(RCC->CSR), RCC_CSR_LSION);
    set_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (false == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
    set_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::lsi));
}

void c_mcu::disable_msi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    while (true == is_flag(RCC->CR, RCC_CR_MSIRDY));
    clear_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::msi));
}

void c_mcu::disable_hsi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_HSION);

    while (true == is_flag(RCC->CR, RCC_CR_HSIRDY));
    clear_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::hsi));
}

void c_mcu::disable_lsi_clock()
{
    clear_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (true == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
    clear_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::lsi));
}

void c_mcu::enable_pll(e_pll_clock_source a_source, const s_pll_config& a_pll_config)
{
    _unused(a_source);
    _assert(true == this->is_clock_enabled(e_clock::hsi));
    _assert(s_pll_config::e_pll_divider::unknown != a_pll_config.pll_divider);
    _assert(s_pll_config::e_pll_multiplier::unknown != a_pll_config.pll_multiplier);

    if (true == a_pll_config.hsi_divader)
    {
        set_flag(&(RCC->CR), RCC_CR_HSIDIVEN);
    }
    else
    {
        clear_flag(&(RCC->CR), RCC_CR_HSIDIVEN);
    }

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY));

    clear_flag(&(RCC->CFGR), RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV);
    set_flag(&(RCC->CFGR), RCC_CFGR_PLLSRC_HSI                           |
                           static_cast<uint32>(a_pll_config.pll_divider) |
                           static_cast<uint32>(a_pll_config.pll_multiplier));

    set_flag(&(RCC->CR), RCC_CR_PLLON);

    while (false == is_flag(RCC->CR, RCC_CR_PLLRDY));
    set_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::pll));
}

void c_mcu::disable_pll()
{
    clear_flag(&(RCC->CR), RCC_CR_PLLON);
    while (true == is_flag(RCC->CR, RCC_CR_PLLRDY));
}

void c_mcu::set_sysclk(e_sysclk_source a_source, const s_bus_prescalers& a_prescalers)
{
    switch (a_source)
    {
    case e_sysclk_source::msi:
        break;
    case c_mcu::e_sysclk_source::hsi:
        break;
    case c_mcu::e_sysclk_source::pll:
        break;
    default:
        break;
    }
}

} // namespace stm32l011xx
} // namespace hal
} // namespace cml

#endif // STM32L011xx