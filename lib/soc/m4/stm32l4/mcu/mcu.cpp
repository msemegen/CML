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
using namespace cml::utils;

constexpr std::uint32_t bus_dividers[] = { 1u, 2u, 4u, 8u, 16u, 64u, 128u, 256u, 512u };
constexpr std::uint32_t msi_frequency_hz_lut[] { 100_kHz, 200_kHz, 400_kHz, 800_kHz, 1_MHz,  2_MHz,
                                                 4_MHz,   8_MHz,   16_MHz,  24_MHz,  32_MHz, 48_MHz };

template<typename Config_t>
uint32_t get_PLL_register_config_from_factor(const Config_t& a_config, uint32_t a_enable_flag)
{
    return (static_cast<uint32_t>(a_config.divider) |
            (rcc<mcu>::PLL::RQP::Output::enabled == a_config.output ? a_enable_flag : 0));
}

#if defined(SOC_PLLSAI_PRESENT)
template<typename Config_t>
uint32_t get_PLLSAI1_register_config_from_factor(const Config_t& a_config, uint32_t a_enable_flag)
{
    return (static_cast<uint32_t>(a_config.divider) |
            (rcc<mcu>::PLL::RQPSAI1::Output::enabled == a_config.output ? a_enable_flag : 0));
}

void enable_PLL(std::uint32_t a_source,
                rcc<mcu>::PLL::M a_m,
                const rcc<mcu>::PLL::RQP a_rqp,
                const rcc<mcu>::PLL::RQPSAI1& a_rqp_sai1)
{
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::R::Divider>() != a_rqp.r.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::Output>() != a_rqp.r.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::Q::Divider>() != a_rqp.q.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::Output>() != a_rqp.q.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::P::Divider>() != a_rqp.p.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::Output>() != a_rqp.p.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::R::Divider>() != a_rqp_sai1.r.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::Output>() != a_rqp_sai1.r.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::Q::Divider>() != a_rqp_sai1.q.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::Output>() != a_rqp_sai1.q.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::P::Divider>() != a_rqp_sai1.p.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::Output>() != a_rqp_sai1.p.output);

    cml_assert(a_rqp.n >= 8 && a_rqp.n <= 86);
    cml_assert(a_rqp_sai1.n >= 8 && a_rqp_sai1.n <= 86);

    RCC->PLLCFGR = a_source | static_cast<std::uint32_t>(a_m) | (a_rqp.n << RCC_PLLCFGR_PLLN_Pos) |
                   get_PLL_register_config_from_factor(a_rqp.p, RCC_PLLCFGR_PLLPEN) |
                   get_PLL_register_config_from_factor(a_rqp.q, RCC_PLLCFGR_PLLQEN) |
                   get_PLL_register_config_from_factor(a_rqp.r, RCC_PLLCFGR_PLLREN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLON);
    wait_until::all_bits(&(RCC->CR), RCC_CR_PLLRDY, false);

    RCC->PLLSAI1CFGR = (a_rqp_sai1.n << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                       get_PLLSAI1_register_config_from_factor(a_rqp_sai1.p, RCC_PLLSAI1CFGR_PLLSAI1PEN) |
                       get_PLLSAI1_register_config_from_factor(a_rqp_sai1.q, RCC_PLLSAI1CFGR_PLLSAI1QEN) |
                       get_PLLSAI1_register_config_from_factor(a_rqp_sai1.r, RCC_PLLSAI1CFGR_PLLSAI1REN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLSAI1ON);
    wait_until::all_bits(&(RCC->CR), RCC_CR_PLLSAI1RDY, false);
}
#endif

#if !defined(SOC_PLLSAI_PRESENT)
void enable_PLL(std::uint32_t a_source, rcc<mcu>::PLL::M a_m, const rcc<mcu>::PLL::RQP a_rqp)
{
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::R::Divider>() != a_rqp.r.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::Output>() != a_rqp.r.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::Q::Divider>() != a_rqp.q.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::Output>() != a_rqp.q.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::P::Divider>() != a_rqp.p.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQP::Output>() != a_rqp.p.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::R::Divider>() != a_rqp_sai1.r.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::Output>() != a_rqp_sai1.r.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::Q::Divider>() != a_rqp_sai1.q.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::Output>() != a_rqp_sai1.q.output);

    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::P::Divider>() != a_rqp_sai1.p.divider);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::PLL::RQPSAI1::Output>() != a_rqp_sai1.p.output);

    cml_assert(a_rqp.n >= 8 && a_rqp.n <= 86);
    cml_assert(a_rqp_sai1.n >= 8 && a_rqp_sai1.n <= 86);

    RCC->PLLCFGR = a_source | static_cast<std::uint32_t>(a_m) | (a_rqp.n << RCC_PLLCFGR_PLLN_Pos) |
                   get_PLL_register_config_from_factor(a_rqp.p, RCC_PLLCFGR_PLLPEN) |
                   get_PLL_register_config_from_factor(a_rqp.q, RCC_PLLCFGR_PLLQEN) |
                   get_PLL_register_config_from_factor(a_rqp.r, RCC_PLLCFGR_PLLREN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLON);
    wait_until::all_bits(&(RCC->CR), RCC_CR_PLLRDY, false);
}
#endif

std::uint32_t calculate_PLL_R_output_frequency()
{
    const std::uint32_t m =
        (static_cast<std::uint32_t>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const std::uint32_t n =
        (static_cast<std::uint32_t>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN)) >> RCC_PLLCFGR_PLLN_Pos);

    std::uint32_t pllvco = 0;

    switch (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC))
    {
        case RCC_PLLCFGR_PLLSRC_MSI: {
            pllvco = (msi_frequency_hz_lut[bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos] / m) * n;
        }
        break;

        case RCC_PLLCFGR_PLLSRC_HSI: {
            pllvco = (16_MHz / m) * n;
        }
        break;
    }

    cml_assert(pllvco >= 96_MHz && pllvco <= 344_MHz);

    std::uint32_t pllr = ((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1u) * 2u;
    return pllvco / pllr;
}
std::uint32_t calculate_PLL_Q_output_frequency()
{
    const std::uint32_t m = (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const std::uint32_t n = bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
    const std::uint32_t q = ((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos) * 2u) + 2u;

    switch (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC))
    {
        case RCC_PLLCFGR_PLLSRC_MSI: {
            return ((rcc<mcu>::MSI::get_frequency_Hz() / m) * n) / q;
        }

        case RCC_PLLCFGR_PLLSRC_HSI: {
            return ((rcc<mcu>::HSI16::get_frequency_Hz() / m) * n) / q;
        }
    }

    return 0;
}

#if defined(SOC_PLLSAI_PRESENT)
std::uint32_t calculate_PLLSAI1_Q_output_frequency()
{
    const std::uint32_t m = (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const std::uint32_t n =
        (bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos) + 1u;
    const std::uint32_t q =
        ((bit_flag::get(RCC->PLLCFGR, RCC_PLLSAI1CFGR_PLLSAI1Q) >> RCC_PLLSAI1CFGR_PLLSAI1Q_Pos) * 2u) + 2u;

    switch (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC))
    {
        case RCC_PLLCFGR_PLLSRC_MSI: {
            return ((rcc<mcu>::MSI::get_frequency_Hz() / m) * n) / q;
        }

        case RCC_PLLCFGR_PLLSRC_HSI: {
            return ((rcc<mcu>::HSI16::get_frequency_Hz() / m) * n) / q;
        }
    }

    return 0;
}
#endif

void set_SYSCLK_source(std::uint32_t a_source, const rcc<mcu>::Bus_prescalers& a_prescalers)
{
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::Bus_prescalers::AHB>() != a_prescalers.ahb);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::Bus_prescalers::APB1>() != a_prescalers.apb1);
    cml_assert(various::get_enum_incorrect_value<rcc<mcu>::Bus_prescalers::APB2>() != a_prescalers.apb2);

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_HSI);
    wait_until::all_bits(&(RCC->CFGR), RCC_CFGR_SW_HSI << RCC_CFGR_SW_Pos, false);

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<std::uint32_t>(a_prescalers.ahb));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<std::uint32_t>(a_prescalers.apb1));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<std::uint32_t>(a_prescalers.apb2));
}
} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {
using namespace cml;
using namespace cml::utils;

void rcc<mcu>::MSI::enable(rcc<mcu>::MSI::Frequency a_frequency)
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);
    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, true);

    bit_flag::set(&(RCC->CR), RCC_CR_MSIRANGE, static_cast<std::uint32_t>(a_frequency) << RCC_CR_MSIRANGE_Pos);
    bit_flag::set(&(RCC->CR), RCC_CR_MSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);

    bit_flag::clear(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);
    bit_flag::set(&(RCC->CR), RCC_CR_MSIRGSEL);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);
}
void rcc<mcu>::MSI::disable()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);
    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, true);
}
std::uint32_t rcc<mcu>::MSI::get_frequency_Hz()
{
    if (true == is_enabled())
    {
        return msi_frequency_hz_lut[bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos];
    }

    return 0u;
}

void rcc<mcu>::HSI16::enable(rcc<mcu>::HSI16::Frequency)
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSION, RCC_CR_HSION);
    wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, false);
}
void rcc<mcu>::HSI16::disable()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_HSION);
    wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, true);
}
std::uint32_t rcc<mcu>::HSI16::get_frequency_Hz()
{
    if (true == is_enabled())
    {
        return 16_MHz;
    }

    return 0u;
}

void rcc<mcu>::HSI48::enable(rcc<mcu>::HSI48::Frequency)
{
    bit_flag::set(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, RCC_CRRCR_HSI48ON);
    wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48RDY, false);
}
void rcc<mcu>::HSI16::disable()
{
    bit_flag::clear(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);
    wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48RDY, true);
}

#if defined(SOC_PLLSAI_PRESENT)
template<> void rcc<mcu>::PLL::enable<rcc<mcu>::MSI>(M a_M, const RQP a_RQP, const RQPSAI1& a_RQPSAI1)
{
    enable_PLL(RCC_PLLCFGR_PLLSRC_MSI, a_M, a_RQP, a_RQPSAI1);
}

template<> void rcc<mcu>::PLL::enable<rcc<mcu>::HSI16>(M a_M, const RQP a_RQP, const RQPSAI1& a_RQPSAI1)
{
    enable_PLL(RCC_PLLCFGR_PLLSRC_HSI, a_M, a_RQP, a_RQPSAI1);
}
#endif
#if !defined(SOC_PLLSAI_PRESENT)
template<> void rcc<mcu>::PLL::enable<rcc<mcu>::MSI>(M a_m, const RQP a_rqp)
{
    enable_PLL(RCC_PLLCFGR_PLLSRC_MSI, a_M, a_RQP);
}

template<> void rcc<mcu>::PLL::enable<rcc<mcu>::HSI>(M a_m, const RQP a_rqp)
{
    enable_PLL(RCC_PLLCFGR_PLLSRC_HSI, a_M, a_RQP);
}
#endif
void rcc<mcu>::PLL::disable()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_PLLON);
    wait_until::all_bits(&(RCC->CR), RCC_CR_PLLRDY, true);

#if defined(SOC_PLLSAI_PRESENT)
    bit_flag::clear(&(RCC->CR), RCC_CR_PLLSAI1ON);
    wait_until::all_bits(&(RCC->CR), RCC_CR_PLLSAI1RDY, true);
#endif
}
std::uint32_t rcc<mcu>::PLL::get_frequency_Hz()
{
    if (true == is_enabled())
    {
        return calculate_PLL_R_output_frequency();
    }

    return 0u;
}

void rcc<mcu>::LSI::enable(Frequency)
{
    bit_flag::set(&(RCC->CSR), RCC_CSR_LSION, RCC_CSR_LSION);
    wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, false);
}
void rcc<mcu>::LSI::disable()
{
    bit_flag::clear(&(RCC->CSR), RCC_CSR_LSION);
    wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, true);
}

template<> void rcc<mcu>::CLK48_mux::set_source<rcc<mcu>::HSI48>()
{
    cml_assert(true == HSI48::is_enabled());

    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL);
}
template<> void rcc<mcu>::CLK48_mux::set_source<rcc<mcu>::MSI>()
{
    cml_assert(true == MSI::is_enabled());

    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1);
}
template<> void rcc<mcu>::CLK48_mux::set_source<rcc<mcu>::PLL::RQP::Q>()
{
    cml_assert(true == MSI::is_enabled());

    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_1);
}
#if defined(SOC_PLLSAI_PRESENT)
template<> void rcc<mcu>::CLK48_mux::set_source<rcc<mcu>::PLL::RQPSAI1::Q>()
{
    cml_assert(true == MSI::is_enabled());

    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0);
}
#endif
std::uint32_t rcc<mcu>::CLK48_mux::get_frequency_Hz()
{
    switch (bit_flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL))
    {
        case 0X0u: {
            return HSI48::get_frequency_Hz();
        }
        break;
#if defined(SOC_PLLSAI_PRESENT)
        case RCC_CCIPR_CLK48SEL_0: {
            return calculate_PLLSAI1_Q_output_frequency();
        }
        break;
#endif
        case RCC_CCIPR_CLK48SEL_1: {
            return calculate_PLL_Q_output_frequency();
        }
        break;

        case RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1: {
            return MSI::get_frequency_Hz();
        }
        break;
    }

    return 0;
}

template<> void rcc<mcu>::set_SYSCLK_source<rcc<mcu>::MSI>(const Bus_prescalers& a_prescalers)
{
    cml_assert(true == MSI::is_enabled());

    ::set_SYSCLK_source(RCC_CFGR_SW_MSI, a_prescalers);

    SystemCoreClock = MSI::get_frequency_Hz();
}
template<> void rcc<mcu>::set_SYSCLK_source<rcc<mcu>::HSI16>(const Bus_prescalers& a_prescalers)
{
    cml_assert(true == HSI16::is_enabled());

    ::set_SYSCLK_source(RCC_CFGR_SW_HSI, a_prescalers);

    SystemCoreClock = HSI16::get_frequency_Hz();
}
template<> void rcc<mcu>::set_SYSCLK_source<rcc<mcu>::PLL>(const Bus_prescalers& a_prescalers)
{
    cml_assert(true == PLL::is_enabled());

    ::set_SYSCLK_source(RCC_CFGR_SW_PLL, a_prescalers);

    SystemCoreClock = PLL::get_frequency_Hz();
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
} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif