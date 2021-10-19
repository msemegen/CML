/*
 *   Name: mcu.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/mcu/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace {

using namespace cml;
using namespace soc::m4;
using namespace soc::m4::stm32l4;

constexpr std::uint32_t bus_dividers[] = { 1u, 2u, 4u, 8u, 16u, 64u, 128u, 256u, 512u };

template<typename Config_t>
uint32_t get_pll_register_config_from_factor(const Config_t& a_config, uint32_t a_enable_flag)
{
    return (static_cast<uint32_t>(a_config.divider) |
            (rcc<mcu>::PLL_config::Output::enabled == a_config.output ? a_enable_flag : 0));
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<typename Config_t>
uint32_t get_pllsai1_register_config_from_factor(const Config_t& a_config, uint32_t a_enable_flag)
{
    return (static_cast<uint32_t>(a_config.divider) |
            (rcc<mcu>::PLLSAI1_config::Output::enabled == a_config.output ? a_enable_flag : 0));
}
#endif

} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;
using namespace cml::utils;

template<> void rcc<mcu>::enable_clock<rcc<mcu>::Clock::MSI>(rcc<mcu>::MSI_frequency a_freq)
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);

    bit_flag::set(&(RCC->CR), RCC_CR_MSIRANGE, static_cast<std::uint32_t>(a_freq) << RCC_CR_MSIRANGE_Pos);

    bit_flag::set(&(RCC->CR), RCC_CR_MSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);

    bit_flag::clear(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);
    bit_flag::set(&(RCC->CR), RCC_CR_MSIRGSEL);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);
}

template<> void rcc<mcu>::enable_clock<rcc<mcu>::Clock::HSI>(HSI_frequency)
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSION, RCC_CR_HSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, false);
}

template<> void rcc<mcu>::enable_clock<rcc<mcu>::Clock::LSI>(LSI_frequency)
{
    bit_flag::set(&(RCC->CSR), RCC_CSR_LSION, RCC_CSR_LSION);

    wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, false);
}

template<> void rcc<mcu>::enable_clock<rcc<mcu>::Clock::HSI48>(HSI48_frequency)
{
    bit_flag::set(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);

    wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, false);
}

template<> void rcc<mcu>::enable_clock<rcc<mcu>::Clock::PLL>(PLL_source a_source, PLLM a_m, const PLL_config& a_config)
{
    cml_assert(various::get_enum_incorrect_value<PLL_config::R::Divider>() != a_config.r.divider);
    cml_assert(various::get_enum_incorrect_value<PLL_config::Output>() != a_config.r.output);

    cml_assert(various::get_enum_incorrect_value<PLL_config::Q::Divider>() != a_config.q.divider);
    cml_assert(various::get_enum_incorrect_value<PLL_config::Output>() != a_config.q.output);

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    cml_assert(various::get_enum_incorrect_value<PLL_config::P::Divider>() != a_config.p.divider);
    cml_assert(various::get_enum_incorrect_value<PLL_config::Output>() != a_config.p.output);
#endif

    cml_assert((true == is_clock_enabled(Clock::MSI) && PLL_source::MSI == a_source) ||
               (true == is_clock_enabled(Clock::HSI) && PLL_source::HSI == a_source));
    cml_assert(a_config.n >= 8 && a_config.n <= 86);

    disable_clock(Clock::PLL);

    RCC->PLLCFGR = static_cast<std::uint32_t>(a_source) | static_cast<std::uint32_t>(a_m) |
                   (a_config.n << RCC_PLLCFGR_PLLN_Pos) |
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
                   get_pll_register_config_from_factor(a_config.p, RCC_PLLCFGR_PLLPEN) |
#endif
                   get_pll_register_config_from_factor(a_config.q, RCC_PLLCFGR_PLLQEN) |
                   get_pll_register_config_from_factor(a_config.r, RCC_PLLCFGR_PLLREN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLON);
    wait_until::all_bits(&(RCC->CR), RCC_CR_PLLRDY, false);
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<mcu>::enable_clock<rcc<mcu>::Clock::PLL>(PLL_source a_source,
                                                             PLLM a_m,
                                                             const PLL_config& a_pll_config,
                                                             const PLLSAI1_config& a_pllsai1_config)
{
    cml_assert(various::get_enum_incorrect_value<PLL_config::R::Divider>() != a_pll_config.r.divider);
    cml_assert(various::get_enum_incorrect_value<PLL_config::Output>() != a_pll_config.r.output);

    cml_assert(various::get_enum_incorrect_value<PLL_config::Q::Divider>() != a_pll_config.q.divider);
    cml_assert(various::get_enum_incorrect_value<PLL_config::Output>() != a_pll_config.q.output);

    cml_assert(various::get_enum_incorrect_value<PLL_config::P::Divider>() != a_pll_config.p.divider);
    cml_assert(various::get_enum_incorrect_value<PLL_config::Output>() != a_pll_config.p.output);

    cml_assert(various::get_enum_incorrect_value<PLLSAI1_config::R::Divider>() != a_pllsai1_config.r.divider);
    cml_assert(various::get_enum_incorrect_value<PLLSAI1_config::Output>() != a_pllsai1_config.r.output);

    cml_assert(various::get_enum_incorrect_value<PLLSAI1_config::Q::Divider>() != a_pllsai1_config.q.divider);
    cml_assert(various::get_enum_incorrect_value<PLLSAI1_config::Output>() != a_pllsai1_config.q.output);

    cml_assert(various::get_enum_incorrect_value<PLLSAI1_config::P::Divider>() != a_pllsai1_config.p.divider);
    cml_assert(various::get_enum_incorrect_value<PLLSAI1_config::Output>() != a_pllsai1_config.p.output);

    cml_assert(a_pll_config.n >= 8 && a_pll_config.n <= 86);
    cml_assert(a_pllsai1_config.n >= 8 && a_pllsai1_config.n <= 86);

    RCC->PLLCFGR = static_cast<std::uint32_t>(a_source) | static_cast<std::uint32_t>(a_m) |
                   (a_pll_config.n << RCC_PLLCFGR_PLLN_Pos) |
                   get_pll_register_config_from_factor(a_pll_config.p, RCC_PLLCFGR_PLLPEN) |
                   get_pll_register_config_from_factor(a_pll_config.q, RCC_PLLCFGR_PLLQEN) |
                   get_pll_register_config_from_factor(a_pll_config.r, RCC_PLLCFGR_PLLREN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLON);
    wait_until::all_bits(&(RCC->CR), RCC_CR_PLLRDY, false);

    RCC->PLLSAI1CFGR = (a_pllsai1_config.n << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                       get_pllsai1_register_config_from_factor(a_pllsai1_config.p, RCC_PLLSAI1CFGR_PLLSAI1PEN) |
                       get_pllsai1_register_config_from_factor(a_pllsai1_config.q, RCC_PLLSAI1CFGR_PLLSAI1QEN) |
                       get_pllsai1_register_config_from_factor(a_pllsai1_config.r, RCC_PLLSAI1CFGR_PLLSAI1REN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLSAI1ON);
    wait_until::all_bits(&(RCC->CR), RCC_CR_PLLSAI1RDY, false);
}
#endif

void rcc<mcu>::disable_clock(Clock a_clock)
{
    switch (a_clock)
    {
        case rcc<mcu>::Clock::MSI: {
            bit_flag::clear(&(RCC->CR), RCC_CR_MSION);
            wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, true);
        }
        break;
        case rcc<mcu>::Clock::HSI: {
            bit_flag::clear(&(RCC->CR), RCC_CR_HSION);
            wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, true);
        }
        break;
        case rcc<mcu>::Clock::PLL: {
            bit_flag::clear(&(RCC->CR), RCC_CR_PLLON);
            wait_until::all_bits(&(RCC->CR), RCC_CR_PLLRDY, true);

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
            bit_flag::clear(&(RCC->CR), RCC_CR_PLLSAI1ON);
            wait_until::all_bits(&(RCC->CR), RCC_CR_PLLSAI1RDY, true);
#endif
        }
        break;
        case rcc<mcu>::Clock::HSI48: {
            bit_flag::clear(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);
            wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, true);
        }
        break;
        case rcc<mcu>::Clock::LSI: {
            bit_flag::clear(&(RCC->CSR), RCC_CSR_LSION);
            wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, true);
        }
        break;
    }
}

void rcc<mcu>::set_MCO(const MCO_config& a_config)
{
    cml_assert(various::get_enum_incorrect_value<MCO_config::Mode>() != a_config.mode);
    cml_assert(various::get_enum_incorrect_value<MCO_config::Source>() != a_config.source);
    cml_assert(various::get_enum_incorrect_value<MCO_config::Divider>() != a_config.divider);

    if (MCO_config::Mode::enabled == a_config.mode)
    {
        cml_assert(MCO_config::Divider::none != a_config.divider);
        cml_assert(MCO_config::Source::none != a_config.source);

        bit_flag::set(&(RCC->CFGR),
                      RCC_CFGR_MCOPRE | RCC_CFGR_MCOSEL,
                      static_cast<std::uint32_t>(a_config.source) | static_cast<std::uint32_t>(a_config.divider));
    }
    else
    {
        cml_assert(MCO_config::Divider::none == a_config.divider);
        cml_assert(MCO_config::Source::none == a_config.source);

        bit_flag::clear(&(RCC->CFGR), RCC_CFGR_MCOPRE | RCC_CFGR_MCOSEL);
    }
}

void rcc<mcu>::set_SYSCFG_active(bool a_active)
{
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_SYSCFGEN, true == a_active ? RCC_APB2ENR_SYSCFGEN : 0x0u);
}

rcc<mcu>::MCO_config rcc<mcu>::get_MCO_config()
{
    std::uint32_t mcosel = bit_flag::get(RCC->CFGR, RCC_CFGR_MCOSEL);

    if (0 != mcosel)
    {
        return { MCO_config::Mode::enabled,
                 static_cast<MCO_config::Source>(mcosel),
                 static_cast<MCO_config::Divider>(READ_BIT(RCC->CFGR, RCC_CFGR_MCOPRE)) };
    }
    else
    {
        return { MCO_config::Mode::disabled, MCO_config::Source::none, MCO_config::Divider::none };
    }
}

bool rcc<mcu>::is_SYSCFG_active()
{
    return bit_flag::get(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
}

void rcc<mcu>::set_CLK48_source(CLK48_source a_source)
{
    cml_assert((CLK48_source::HSI48 == a_source && true == is_clock_enabled(Clock::HSI48)) ||
               (CLK48_source::MSI == a_source && true == is_clock_enabled(Clock::MSI)) ||

               ((CLK48_source::PLL_Q == a_source && true == is_clock_enabled(Clock::PLL)) &&
                PLL_config::Output::enabled == get_PLL_config().q.output)
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
               || ((CLK48_source::PLL_SAI1_Q == a_source && is_clock_enabled(Clock::PLL)) &&
                   PLLSAI1_config::Output::enabled == get_PLLSAI1_config().q.output)
#endif
    );

    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, static_cast<std::uint32_t>(a_source));
}

void rcc<mcu>::set_SYSCLK_source(SYSCLK_source a_source, const Bus_prescalers& a_prescalers)
{
    cml_assert(various::get_enum_incorrect_value<Bus_prescalers::AHB>() != a_prescalers.ahb);
    cml_assert(various::get_enum_incorrect_value<Bus_prescalers::APB1>() != a_prescalers.apb1);
    cml_assert(various::get_enum_incorrect_value<Bus_prescalers::APB2>() != a_prescalers.apb2);

    std::uint32_t frequency_hz = 0;

    switch (a_source)
    {
        case SYSCLK_source::HSI: {
            cml_assert(true == is_clock_enabled(Clock::HSI));
            frequency_hz = get_clock_frequency_Hz(Clock::HSI);
        }
        break;

        case SYSCLK_source::MSI: {
            cml_assert(true == is_clock_enabled(Clock::MSI));
            frequency_hz = get_clock_frequency_Hz(Clock::MSI);
        }
        break;

        case SYSCLK_source::PLL: {
            cml_assert(true == is_clock_enabled(Clock::PLL));
            frequency_hz = calculate_PLL_R_output_frequency();
        }
        break;
    }

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, static_cast<std::uint32_t>(a_source));
    wait_until::all_bits(&(RCC->CFGR), static_cast<std::uint32_t>(a_source) << RCC_CFGR_SW_Pos, false);

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<std::uint32_t>(a_prescalers.ahb));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<std::uint32_t>(a_prescalers.apb1));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<std::uint32_t>(a_prescalers.apb2));

    SystemCoreClock = frequency_hz;
}

rcc<mcu>::Bus_prescalers rcc<mcu>::get_bus_prescalers()
{
    return { static_cast<Bus_prescalers::AHB>(bit_flag::get(RCC->CFGR, RCC_CFGR_HPRE)),
             static_cast<Bus_prescalers::APB1>(bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE1)),
             static_cast<Bus_prescalers::APB2>(bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE2)) };
}

rcc<mcu>::PLL_config rcc<mcu>::get_PLL_config()
{
    return
    {
        bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN),
            { static_cast<PLL_config::R::Divider>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLR)),
              static_cast<PLL_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN)) },
            { static_cast<PLL_config::Q::Divider>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ)),
              static_cast<PLL_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN)) },
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
            { static_cast<PLL_config::P::Divider>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLP)),
              static_cast<PLL_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN)) },
#endif
    };
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
rcc<mcu>::PLLSAI1_config rcc<mcu>::get_PLLSAI1_config()
{
    return { bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N),
             { static_cast<PLLSAI1_config::R::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R)),
               static_cast<PLLSAI1_config::Output>(bit_flag::is(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1REN)) },
             { static_cast<PLLSAI1_config::Q::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q)),
               static_cast<PLLSAI1_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLSAI1CFGR_PLLSAI1QEN)) },
             { static_cast<PLLSAI1_config::P::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P)),
               static_cast<PLLSAI1_config::Output>(bit_flag::is(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PEN)) } };
}
#endif

std::uint32_t rcc<mcu>::get_clock_frequency_Hz(Clock a_clock)
{
    static constexpr std::uint32_t msi_frequency_hz_lut[] { 100_kHz, 200_kHz, 400_kHz, 800_kHz, 1_MHz,  2_MHz,
                                                            4_MHz,   8_MHz,   16_MHz,  24_MHz,  32_MHz, 48_MHz };

    switch (a_clock)
    {
        case Clock::MSI: {
            return msi_frequency_hz_lut[bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos];
        }
        break;
        case rcc<mcu>::Clock::HSI: {
            return 16_MHz;
        }
        break;
        case rcc<mcu>::Clock::PLL: {
            return calculate_PLL_R_output_frequency();
        }
        break;
        case rcc<mcu>::Clock::HSI48: {
            return 48_MHz;
        }
        break;
        case rcc<mcu>::Clock::LSI: {
            return 32_MHz;
        }
        break;
    }

    return 0;
}

std::uint32_t rcc<mcu>::get_CLK48_frequency_Hz()
{
    CLK48_source source = get_CLK48_source();

    switch (source)
    {
        case CLK48_source::HSI48: {
            return get_clock_frequency_Hz(Clock::HSI48);
        }
        break;
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case CLK48_source::PLL_SAI1_Q: {
            return calculate_PLLSAI1_Q_output_frequency();
        }
        break;
#endif
        case rcc::CLK48_source::PLL_Q: {
            return calculate_PLL_Q_output_frequency();
        }
        break;

        case rcc::CLK48_source::MSI: {
            return get_clock_frequency_Hz(Clock::MSI);
        }
        break;
    }

    return 0;
}

std::uint32_t rcc<mcu>::get_SYSCLK_frequency_Hz()
{
    return SystemCoreClock;
}

std::uint32_t rcc<mcu>::get_HCLK_frequency_Hz()
{
    std::uint32_t index = bit_flag::get(RCC->CFGR, RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;

    if (index > 0)
    {
        return SystemCoreClock / (bus_dividers[index - 8u]);
    }

    return SystemCoreClock;
}

std::uint32_t rcc<mcu>::get_PCLK1_frequency_Hz()
{
    std::uint32_t index = bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;

    if (index > 0)
    {
        return SystemCoreClock / (bus_dividers[index - 8u]);
    }

    return SystemCoreClock;
}

std::uint32_t rcc<mcu>::get_PCLK2_frequency_Hz()
{
    std::uint32_t index = bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;

    if (index > 0)
    {
        return SystemCoreClock / (bus_dividers[index - 8u]);
    }

    return SystemCoreClock;
}

bool rcc<mcu>::is_clock_enabled(Clock a_clock)
{
    switch (a_clock)
    {
        case Clock::MSI:
        case Clock::HSI:
        case Clock::PLL: {
            return bit_flag::is(RCC->CR, static_cast<std::uint32_t>(a_clock));
        }
        break;

        case Clock::LSI: {
            return bit_flag::is(RCC->CSR, RCC_CSR_LSION);
        }
        break;

        case Clock::HSI48: {
            return bit_flag::is(RCC->CRRCR, RCC_CRRCR_HSI48ON);
        }
        break;
    }

    return false;
}

rcc<mcu>::Reset_source rcc<mcu>::get_reset_source()
{
    std::uint32_t flag = bit_flag::get(RCC->CSR, 0xFB000000u);

    if (flag == 0x0u)
    {
        flag = RCC_CSR_PINRSTF;
    }

    bit_flag::set(&(RCC->CSR), RCC_CSR_RMVF);

    return static_cast<Reset_source>(flag);
}

std::uint32_t rcc<mcu>::calculate_PLL_R_output_frequency()
{
    const std::uint32_t m =
        (static_cast<std::uint32_t>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const std::uint32_t n =
        (static_cast<std::uint32_t>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN)) >> RCC_PLLCFGR_PLLN_Pos);

    std::uint32_t pllvco = 0;

    switch (static_cast<PLL_source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case PLL_source::MSI: {
            pllvco = (get_clock_frequency_Hz(Clock::MSI) / m) * n;
        }
        break;

        case PLL_source::HSI: {
            pllvco = (get_clock_frequency_Hz(Clock::HSI) / m) * n;
        }
        break;
    }

    cml_assert(pllvco >= 96_MHz && pllvco <= 344_MHz);

    std::uint32_t pllr = ((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1u) * 2u;
    return pllvco / pllr;
}

std::uint32_t rcc<mcu>::calculate_PLL_Q_output_frequency()
{
    const std::uint32_t m = (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const std::uint32_t n = bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
    const std::uint32_t q = ((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos) * 2u) + 2u;

    switch (static_cast<PLL_source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case PLL_source::MSI: {
            return ((get_clock_frequency_Hz(Clock::MSI) / m) * n) / q;
        }

        case PLL_source::HSI: {
            return ((get_clock_frequency_Hz(Clock::HSI) / m) * n) / q;
        }
    }

    return 0;
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
std::uint32_t rcc<mcu>::calculate_PLLSAI1_Q_output_frequency()
{
    const std::uint32_t m = (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const std::uint32_t n =
        (bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos) + 1u;
    const std::uint32_t q =
        ((bit_flag::get(RCC->PLLCFGR, RCC_PLLSAI1CFGR_PLLSAI1Q) >> RCC_PLLSAI1CFGR_PLLSAI1Q_Pos) * 2u) + 2u;

    switch (static_cast<PLL_source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case PLL_source::MSI: {
            return ((get_clock_frequency_Hz(Clock::MSI) / m) * n) / q;
        }

        case PLL_source::HSI: {
            return ((get_clock_frequency_Hz(Clock::HSI) / m) * n) / q;
        }
    }

    return 0;
}
#endif

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4