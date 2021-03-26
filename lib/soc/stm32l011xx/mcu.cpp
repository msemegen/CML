/*
 *   Name: mcu.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l011xx;

constexpr uint32_t msi_frequency_lut[] { 65536u, 131072u, 262144u, 524288u, 1048u * 1000u, 2097 * 1000u, 4194 * 1000u };

mcu::Sysclk_frequency_change_callback pre_sysclk_frequency_change_callback;
mcu::Sysclk_frequency_change_callback post_sysclk_frequency_change_callback;

} // namespace

namespace soc {
namespace stm32l011xx {

using namespace cml;

void mcu::enable_msi_clock(Msi_frequency a_freq)
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);
    bit_flag::set(&(RCC->ICSCR), RCC_ICSCR_MSIRANGE, static_cast<uint32_t>(a_freq) << RCC_ICSCR_MSIRANGE_Pos);
    bit_flag::set(&(RCC->CR), RCC_CR_MSION);

    while (false == bit_flag::is(RCC->CR, RCC_CR_MSIRDY))
        ;

    bit_flag::clear(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);

    while (false == bit_flag::is(RCC->CR, RCC_CR_MSIRDY))
        ;
}

void mcu::enable_hsi_clock(Hsi_frequency)
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSION);

    while (false == bit_flag::is(RCC->CR, RCC_CR_HSIRDY))
        ;
}

void mcu::enable_lsi_clock(Lsi_frequency)
{
    bit_flag::set(&(RCC->CSR), RCC_CSR_LSION);

    while (false == bit_flag::is(RCC->CSR, RCC_CSR_LSIRDY))
        ;
}

void mcu::disable_msi_clock()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);

    while (true == bit_flag::is(RCC->CR, RCC_CR_MSIRDY))
        ;
}

void mcu::disable_hsi_clock()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_HSION);

    while (true == bit_flag::is(RCC->CR, RCC_CR_HSIRDY))
        ;
}

void mcu::disable_lsi_clock()
{
    bit_flag::clear(&(RCC->CSR), RCC_CSR_LSION);

    while (true == bit_flag::is(RCC->CSR, RCC_CSR_LSIRDY))
        ;
}

void mcu::enable_pll(const Pll_config& a_config)
{
    cml_assert(true == is_clock_enabled(Clock::hsi));
    cml_assert(Pll_config::Divider::unknown != a_config.divider);
    cml_assert(Pll_config::Multiplier::unknown != a_config.multiplier);

    disable_pll();

    if (Pll_config::Source::Divider::_4 == a_config.source.divider)
    {
        bit_flag::set(&(RCC->CR), RCC_CR_HSIDIVEN);
    }
    else
    {
        bit_flag::clear(&(RCC->CR), RCC_CR_HSIDIVEN);
    }

    while (false == bit_flag::is(RCC->CR, RCC_CR_HSIRDY))
        ;

    bit_flag::set(&(RCC->CFGR),
                  RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV,
                  static_cast<uint32_t>(a_config.source.type) | static_cast<uint32_t>(a_config.divider) |
                      static_cast<uint32_t>(a_config.multiplier));

    bit_flag::set(&(RCC->CR), RCC_CR_PLLON);

    while (false == bit_flag::is(RCC->CR, RCC_CR_PLLRDY))
        ;
}

void mcu::disable_pll()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_PLLON);
    while (true == bit_flag::is(RCC->CR, RCC_CR_PLLRDY))
        ;
}

void mcu::set_sysclk(Sysclk_source a_source, const Bus_prescalers& a_prescalers)
{
    if (nullptr != pre_sysclk_frequency_change_callback.function)
    {
        pre_sysclk_frequency_change_callback.function(pre_sysclk_frequency_change_callback.p_user_data);
    }

    if (false == bit_flag::is(RCC->APB1ENR, RCC_APB1ENR_PWREN))
    {
        bit_flag::set(&(RCC->APB1ENR), RCC_APB1ENR_PWREN);
    }

    uint32_t frequency_hz = 0;

    switch (a_source)
    {
        case Sysclk_source::msi: {
            cml_assert(true == is_clock_enabled(Clock::msi));

            frequency_hz = msi_frequency_lut[bit_flag::get(RCC->ICSCR, RCC_ICSCR_MSIRANGE) >> RCC_ICSCR_MSIRANGE_Pos];
        }
        break;

        case mcu::Sysclk_source::hsi: {
            cml_assert(true == is_clock_enabled(Clock::hsi));

            frequency_hz = get_hsi_frequency_hz();
        }
        break;

        case mcu::Sysclk_source::pll: {
            cml_assert(true == is_clock_enabled(Clock::pll));

            frequency_hz = calculate_frequency_from_pll_configuration();
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

    bit_flag::set(&(FLASH->ACR), FLASH_ACR_PRFTEN | FLASH_ACR_PRE_READ);
    bit_flag::clear(&(FLASH->ACR), FLASH_ACR_DISAB_BUF);

    if (nullptr != post_sysclk_frequency_change_callback.function)
    {
        post_sysclk_frequency_change_callback.function(post_sysclk_frequency_change_callback.p_user_data);
    }
}

void mcu::reset()
{
    NVIC_SystemReset();
}

void mcu::halt()
{
    __disable_irq();

    while (true)
        ;
}

void mcu::register_pre_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback)
{
    pre_sysclk_frequency_change_callback = a_callback;
}

void mcu::register_post_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback)
{
    post_sysclk_frequency_change_callback = a_callback;
}

mcu::Bus_prescalers mcu::get_bus_prescalers()
{
    return { static_cast<Bus_prescalers::AHB>(bit_flag::get(RCC->CFGR, RCC_CFGR_HPRE)),
             static_cast<Bus_prescalers::APB1>(bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE1)),
             static_cast<Bus_prescalers::APB2>(bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE2)) };
}

mcu::Pll_config mcu::get_pll_config()
{
    return { { Pll_config::Source::Type::hsi,
               static_cast<Pll_config::Source::Divider>(cml::bit_flag::get(RCC->CR, RCC_CR_HSIDIVEN)) },
             static_cast<Pll_config::Multiplier>(bit_flag::get(RCC->CFGR, RCC_CFGR_PLLMUL)),
             static_cast<Pll_config::Divider>(bit_flag::get(RCC->CFGR, RCC_CFGR_PLLDIV)) };
}

mcu::Flash_latency mcu::select_flash_latency(uint32_t a_syclk_freq, Voltage_scaling a_voltage_scaling)
{
    switch (a_voltage_scaling)
    {
        case Voltage_scaling::_1: {
            if (a_syclk_freq <= 16 * 1000000u)
            {
                return Flash_latency::_0;
            }

            if (a_syclk_freq <= 32 * 1000000u)
            {
                return Flash_latency::_1;
            }
        }
        break;

        case Voltage_scaling::_2: {
            if (a_syclk_freq <= 8 * 1000000u)
            {
                return Flash_latency::_0;
            }

            if (a_syclk_freq <= 16 * 1000000u)
            {
                return Flash_latency::_1;
            }
        }
        break;

        case Voltage_scaling::_3: {
            if (a_syclk_freq <= 4200 * 1000u)
            {
                return Flash_latency::_0;
            }
        }
        break;

        case Voltage_scaling::unknown: {
            cml_assert(a_voltage_scaling != Voltage_scaling::unknown);
        }
        break;
    }

    return Flash_latency::unknown;
}

mcu::Voltage_scaling mcu::select_voltage_scaling(Sysclk_source a_source, uint32_t a_sysclk_freq)
{
    if (Sysclk_source::msi == a_source || (Sysclk_source::pll == a_source && a_sysclk_freq <= 4 * 1000000u))
    {
        return Voltage_scaling::_3;
    }

    if (Sysclk_source::hsi == a_source || (Sysclk_source::pll == a_source && a_sysclk_freq <= 16 * 1000000u))
    {
        return Voltage_scaling::_2;
    }

    if (Sysclk_source::pll == a_source && a_sysclk_freq <= 32 * 1000000u)
    {
        return Voltage_scaling::_1;
    }

    return Voltage_scaling::unknown;
}

void mcu::set_flash_latency(Flash_latency a_latency)
{
    switch (a_latency)
    {
        case Flash_latency::_0: {
            bit_flag::clear(&(FLASH->ACR), FLASH_ACR_LATENCY);
        }
        break;

        case Flash_latency::_1: {
            bit_flag::set(&(FLASH->ACR), FLASH_ACR_LATENCY);
        }
        break;

        case Flash_latency::unknown: {
            cml_assert(Flash_latency::unknown != a_latency);
        }
        break;
    }
}

void mcu::set_voltage_scaling(Voltage_scaling a_scaling)
{
    cml_assert(a_scaling != Voltage_scaling::unknown);

    bit_flag::set(&(PWR->CR), PWR_CR_VOS, static_cast<uint32_t>(a_scaling));
}

void mcu::set_sysclk_source(Sysclk_source a_sysclk_source)
{
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, static_cast<uint32_t>(a_sysclk_source));
    while (false == bit_flag::is(RCC->CFGR, static_cast<uint32_t>(a_sysclk_source) << RCC_CFGR_SWS_Pos))
        ;
}

void mcu::set_bus_prescalers(const Bus_prescalers& a_prescalers)
{
    cml_assert(Bus_prescalers::AHB::unknown != a_prescalers.ahb);
    cml_assert(Bus_prescalers::APB1::unknown != a_prescalers.apb1);
    cml_assert(Bus_prescalers::APB2::unknown != a_prescalers.apb2);

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<uint32_t>(a_prescalers.ahb));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<uint32_t>(a_prescalers.apb1));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<uint32_t>(a_prescalers.apb2));
}

void mcu::increase_sysclk_frequency(Sysclk_source a_source, uint32_t a_frequency_hz, const Bus_prescalers& a_prescalers)
{
    auto new_voltage_scaling = select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = select_flash_latency(a_frequency_hz, new_voltage_scaling);

    cml_assert(Voltage_scaling::unknown != new_voltage_scaling);
    cml_assert(Flash_latency::unknown != new_flash_latency);

    auto current_voltage_scaling = get_voltage_scaling();
    auto current_flash_latency   = get_flash_latency();

    if ((Voltage_scaling::_3 == current_voltage_scaling && Voltage_scaling::_2 == new_voltage_scaling) ||
        (Voltage_scaling::_2 == current_voltage_scaling && Voltage_scaling::_1 == new_voltage_scaling))
    {
        set_voltage_scaling(new_voltage_scaling);
        while (true == bit::is(PWR->CSR, PWR_CSR_VOSF_Pos))
            ;
    }

    if (Flash_latency::_1 == current_flash_latency && Flash_latency::_0 == new_flash_latency)
    {
        set_flash_latency(new_flash_latency);
        while (true == bit::is(FLASH->ACR, FLASH_ACR_LATENCY_Pos))
            ;
    }

    set_sysclk_source(a_source);
    set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

void mcu::decrease_sysclk_frequency(Sysclk_source a_source, uint32_t a_frequency_hz, const Bus_prescalers& a_prescalers)
{
    set_sysclk_source(a_source);

    auto new_voltage_scaling = select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = select_flash_latency(a_frequency_hz, new_voltage_scaling);

    cml_assert(Voltage_scaling::unknown != new_voltage_scaling);
    cml_assert(Flash_latency::unknown != new_flash_latency);

    auto current_voltage_scaling = get_voltage_scaling();
    auto current_flash_latency   = get_flash_latency();

    if ((Voltage_scaling::_2 == current_voltage_scaling && Voltage_scaling::_3 == new_voltage_scaling) ||
        (Voltage_scaling::_1 == current_voltage_scaling && Voltage_scaling::_2 == new_voltage_scaling))
    {
        set_voltage_scaling(new_voltage_scaling);
        while (true == bit::is(PWR->CSR, PWR_CSR_VOSF_Pos))
            ;
    }

    if (Flash_latency::_0 == current_flash_latency && Flash_latency::_1 == new_flash_latency)
    {
        set_flash_latency(new_flash_latency);
        while (false == bit::is(FLASH->ACR, FLASH_ACR_LATENCY_Pos))
            ;
    }

    set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

uint32_t mcu::calculate_frequency_from_pll_configuration()
{
    constexpr uint32_t m_lut[] = { 3, 4, 6, 8, 12, 16, 24, 32, 48 };
    constexpr uint32_t d_lut[] = { 2, 3, 4 };

    const uint32_t mi = (bit_flag::get(RCC->CFGR, RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_Pos) - 0;
    const uint32_t di = (bit_flag::get(RCC->CFGR, RCC_CFGR_PLLDIV) >> RCC_CFGR_PLLDIV_Pos) - 1;

    cml_assert(di < sizeof(d_lut) / sizeof(d_lut[0]));
    cml_assert(mi < sizeof(m_lut) / sizeof(m_lut[0]));

    return get_hsi_frequency_hz() / (true == bit_flag::is(RCC->CR, RCC_CR_HSIDIVEN) ? 4 : 1) * m_lut[mi] / d_lut[di];
}

} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx