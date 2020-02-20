/*
    Name: mcu.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <hal/stm32l452xx/mcu.hpp>

//external
#include <stm32l452xx.h>

//cml
#include <common/assert.hpp>
#include <common/frequency.hpp>
#include <common/macros.hpp>
#include "mcu.hpp"

namespace {

using namespace cml::common;
using namespace cml::hal::stm32l452xx;

constexpr frequency msi_frequency_lut[]
{
    kHz(100u),
    kHz(200u),
    kHz(400u),
    kHz(800u),
    MHz(1u),
    MHz(2u),
    MHz(4u),
    MHz(8u),
    MHz(16u),
    MHz(24u),
    MHz(32u),
    MHz(48u)
};

mcu::Sysclk_frequency_change_callback pre_sysclk_frequency_change_callback;
mcu::Sysclk_frequency_change_callback post_sysclk_frequency_change_callback;

template<typename Config>
uint32 get_pll_register_config_from_factor(const Config& a_config, uint32 a_enable_flag)
{
    return (Config::Divider::unknown != a_config.divider
            ? (static_cast<uint32>(a_config.divider) | (true == a_config.output_enabled
                                                        ? a_enable_flag
                                                        : 0))
            : 0);
}

} // namespace ::

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace common;

void mcu::enable_msi_clock(Msi_frequency a_freq)
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    set_flag(&(RCC->CR), RCC_CR_MSIRANGE, static_cast<uint32>(a_freq) << RCC_CR_MSIRANGE_Pos);

    set_flag(&(RCC->CR), RCC_CR_MSION);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));

    clear_flag(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);
    set_flag(&(RCC->CR), RCC_CR_MSIRGSEL);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));
}

void mcu::enable_hsi_clock(Hsi_frequency a_freq)
{
    unused(a_freq);

    set_flag(&(RCC->CR), RCC_CR_HSION, RCC_CR_HSION);

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY));
}

void mcu::enable_lsi_clock(Lsi_frequency a_freq)
{
    unused(a_freq);

    set_flag(&(RCC->CSR), RCC_CSR_LSION, RCC_CSR_LSION);

    while (false == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
}

void mcu::disable_msi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    while (true == is_flag(RCC->CR, RCC_CR_MSIRDY));
}

void mcu::disable_hsi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_HSION);

    while (true == is_flag(RCC->CR, RCC_CR_HSIRDY));
}

void mcu::disable_lsi_clock()
{
    clear_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (true == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
}

void mcu::enable_pll(const Pll_config& a_config)
{
    assert((true == is_clock_enabled(Clock::msi) && a_config.source == Pll_config::Source::msi) ||
           (true == is_clock_enabled(Clock::hsi) && a_config.source == Pll_config::Source::hsi));
    assert((a_config.pll.n >= 8 && a_config.pll.n <= 86) && (a_config.pllsai1.n >= 8 && a_config.pllsai1.n <= 86));

    disable_pll();

    RCC->PLLCFGR = static_cast<uint32>(a_config.source)    |
                   static_cast<uint32>(a_config.m_divider) |
                   get_pll_register_config_from_factor(a_config.pll.p, RCC_PLLCFGR_PLLPEN) |
                   get_pll_register_config_from_factor(a_config.pll.q, RCC_PLLCFGR_PLLQEN) |
                   get_pll_register_config_from_factor(a_config.pll.r, RCC_PLLCFGR_PLLREN);

    set_flag(&(RCC->CR), RCC_CR_PLLON);
    while (false == get_flag(RCC->CR, RCC_CR_PLLRDY));

    RCC->PLLSAI1CFGR = get_pll_register_config_from_factor(a_config.pllsai1.p, RCC_PLLSAI1CFGR_PLLSAI1PEN) |
                       get_pll_register_config_from_factor(a_config.pllsai1.q, RCC_PLLSAI1CFGR_PLLSAI1QEN) |
                       get_pll_register_config_from_factor(a_config.pllsai1.r, RCC_PLLSAI1CFGR_PLLSAI1REN);

    set_flag(&(RCC->CR), RCC_CR_PLLSAI1ON);
    while (false == get_flag(RCC->CR, RCC_CR_PLLSAI1RDY));
}

void mcu::disable_pll()
{
    clear_flag(&(RCC->CR), RCC_CR_PLLON);
    while (true == is_flag(RCC->CR, RCC_CR_PLLRDY));

    clear_flag(&(RCC->CR), RCC_CR_PLLSAI1ON);
    while (true == is_flag(RCC->CR, RCC_CR_PLLSAI1RDY));
}

void mcu::set_sysclk(Sysclk_source a_source, const Bus_prescalers& a_prescalers, const NVIC_config& a_nvic_settings)
{
    if (nullptr != pre_sysclk_frequency_change_callback.p_function)
    {
        pre_sysclk_frequency_change_callback.p_function(pre_sysclk_frequency_change_callback.a_p_user_data);
    }

    if (false == is_flag(RCC->APB1ENR1, RCC_APB1ENR1_PWREN))
    {
        set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_PWREN);
    }

    if (false == is_flag(RCC->AHB1ENR, RCC_AHB1ENR_FLASHEN))
    {
        set_flag(&(RCC->AHB1ENR), RCC_AHB1ENR_FLASHEN);
    }

    uint32 frequency_hz = 0;

    switch (a_source)
    {
        case Sysclk_source::hsi:
        {
            assert(true == is_clock_enabled(Clock::hsi));

            frequency_hz = config::clock::hsi_frequency_hz;
        }
        break;

        case Sysclk_source::msi:
        {
            assert(true == is_clock_enabled(Clock::msi));

            frequency_hz = msi_frequency_lut[get_flag(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos];
        }
        break;

        case Sysclk_source::pll:
        {
            assert(true == is_clock_enabled(Clock::pll));

            frequency_hz = calculate_pll_frequency();
        }
        break;
    }

    if (SystemCoreClock > frequency_hz)
    {
        decrease_sysclk_frequency(a_source, frequency_hz, a_prescalers);
    }
    else if (SystemCoreClock < frequency_hz)
    {
        increase_sysclk_frequency(a_source, frequency_hz, a_prescalers);
    }

    if (Sysclk_source::pll == a_source)
    {
        set_flag(&(RCC->CFGR), RCC_CFGR_SW_PLL);
        while (false == is_flag(RCC->CFGR, RCC_CFGR_SW_PLL));
    }

    if (Flash_latency::_0 != get_flash_latency())
    {
        set_flag(&(FLASH->ACR), FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_ICEN);
    }

    NVIC_SetPriorityGrouping(a_nvic_settings.priority_grouping);
    __set_BASEPRI(a_nvic_settings.base_priority);

    if (nullptr != post_sysclk_frequency_change_callback.p_function)
    {
        post_sysclk_frequency_change_callback.p_function(post_sysclk_frequency_change_callback.a_p_user_data);
    }
}

void mcu::reset()
{
    NVIC_SystemReset();
}

void mcu::halt()
{
    uint32 new_basepri = 0;

    __asm volatile
    (
        "mov %0, %1      \n" \
        "msr basepri, %0 \n" \
        "isb             \n" \
        "dsb             \n" \
        :"=r" (new_basepri) : "i" (80u)
    );

    while (true);
}

void mcu::register_pre_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback)
{
    pre_sysclk_frequency_change_callback = a_callback;
}

void mcu::register_post_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback)
{
    post_sysclk_frequency_change_callback = a_callback;
}

mcu::Flash_latency mcu::select_flash_latency(uint32 a_syclk_freq,
                                             Voltage_scaling a_voltage_scaling)
{
    switch (a_voltage_scaling)
    {
        case Voltage_scaling::_1:
        {
            if (a_syclk_freq <= MHz(16))
            {
                return Flash_latency::_0;
            }

            if (a_syclk_freq <= MHz(32))
            {
                return Flash_latency::_1;
            }

            if (a_syclk_freq <= MHz(48))
            {
                return Flash_latency::_2;
            }

            if (a_syclk_freq <= MHz(64))
            {
                return Flash_latency::_3;
            }

            if (a_syclk_freq <= MHz(80))
            {
                return Flash_latency::_4;
            }
        }
        break;

        case Voltage_scaling::_2:
        {
            if (a_syclk_freq <= MHz(6))
            {
                return Flash_latency::_0;
            }

            if (a_syclk_freq <= MHz(12))
            {
                return Flash_latency::_1;
            }

            if (a_syclk_freq <= MHz(18))
            {
                return Flash_latency::_2;
            }

            if (a_syclk_freq <= MHz(26))
            {
                return Flash_latency::_3;
            }
        }
        break;

        case Voltage_scaling::unkown:
        {
            assert(Voltage_scaling::unkown != a_voltage_scaling);
        }
        break;
    }

    return Flash_latency::unknown;
}

mcu::Voltage_scaling mcu::select_voltage_scaling(Sysclk_source a_source, uint32 a_sysclk_freq)
{
    if ((Sysclk_source::msi == a_source && a_sysclk_freq <= MHz(24)) ||
        (Sysclk_source::pll == a_source && a_sysclk_freq <= MHz(26)))
    {
        return Voltage_scaling::_2;
    }

    if ((Sysclk_source::msi == a_source && a_sysclk_freq <= MHz(48)) ||
        (Sysclk_source::pll == a_source && a_sysclk_freq <= MHz(80)) ||
        (Sysclk_source::hsi == a_source))
    {
        return Voltage_scaling::_1;
    }

    return Voltage_scaling::unkown;
}

void mcu::set_flash_latency(Flash_latency a_latency)
{
    assert(a_latency != Flash_latency::unknown);

    set_flag(&(FLASH->ACR), FLASH_ACR_LATENCY, static_cast<uint32>(a_latency));

    while (false == is_flag(FLASH->ACR, static_cast<uint32>(a_latency)));
}

void mcu::set_voltage_scaling(Voltage_scaling a_scaling)
{
    assert(a_scaling != Voltage_scaling::unkown);

    set_flag(&(PWR->CR1), PWR_CR1_VOS, static_cast<uint32>(a_scaling));

    while (false == get_bit(PWR->SR2, PWR_SR2_VOSF_Pos));
}

void mcu::set_sysclk_source(Sysclk_source a_sysclk_source)
{
    set_flag(&(RCC->CFGR), RCC_CFGR_SW, static_cast<uint32>(a_sysclk_source));
    while (false == is_flag(RCC->CFGR, static_cast<uint32>(a_sysclk_source) << RCC_CFGR_SWS_Pos));
}

void mcu::set_bus_prescalers(const Bus_prescalers& a_prescalers)
{
    assert(Bus_prescalers::AHB::unknown  != a_prescalers.ahb);
    assert(Bus_prescalers::APB1::unknown != a_prescalers.apb1);
    assert(Bus_prescalers::APB2::unknown != a_prescalers.apb2);

    set_flag(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<uint32>(a_prescalers.ahb));
    set_flag(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<uint32>(a_prescalers.apb1));
    set_flag(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<uint32>(a_prescalers.apb2));
}

void mcu::increase_sysclk_frequency(Sysclk_source a_source,
                                    uint32 a_frequency_hz,
                                    const Bus_prescalers& a_prescalers)
{
    auto new_voltage_scaling = select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = select_flash_latency(a_frequency_hz, new_voltage_scaling);

    assert(Voltage_scaling::unkown != new_voltage_scaling);
    assert(Flash_latency::unknown  != new_flash_latency);

    if (Voltage_scaling::_2 == get_voltage_scaling() && Voltage_scaling::_1 == new_voltage_scaling)
    {
        set_voltage_scaling(new_voltage_scaling);
    }

    if (new_flash_latency != get_flash_latency())
    {
        set_flash_latency(new_flash_latency);
    }

    set_sysclk_source(a_source);
    set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

void mcu::decrease_sysclk_frequency(Sysclk_source a_source,
                                    uint32 a_frequency_hz,
                                    const Bus_prescalers& a_prescalers)
{
    set_sysclk_source(a_source);

    auto new_voltage_scaling = select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = select_flash_latency(a_frequency_hz, new_voltage_scaling);

    assert(Voltage_scaling::unkown != new_voltage_scaling);
    assert(Flash_latency::unknown  != new_flash_latency);

    if (new_flash_latency != get_flash_latency())
    {
        set_flash_latency(new_flash_latency);
    }

    if (Voltage_scaling::_1 == get_voltage_scaling() && Voltage_scaling::_2 == new_voltage_scaling)
    {
        set_voltage_scaling(new_voltage_scaling);
    }

    set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

uint32 mcu::calculate_pll_frequency()
{
    uint32 pllm = (static_cast<uint32>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    uint32 plln = (static_cast<uint32>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLN)) >> RCC_PLLCFGR_PLLN_Pos);

    uint32 pllvco = 0;

    switch (static_cast<Pll_config::Source>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case Pll_config::Source::msi:
        {
            assert(true == is_clock_enabled(Clock::msi));

            uint32 msi_range = get_flag(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            pllvco = (msi_frequency_lut[msi_range] / pllm) * plln;
        }
        break;

        case Pll_config::Source::hsi:
        {
            assert(true == is_clock_enabled(Clock::hsi));

            pllvco = (config::clock::hsi_frequency_hz / pllm) * plln;
        }
        break;

        default:
        {
            assert(false);
        }
        break;
    }

    assert(pllvco >= 96 && pllvco <= 344);

    uint32 pllr = ((get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1u) * 2u;
    return pllvco / pllr;
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml

#endif // STM32L452xx