/*
 *   Name: rcc.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/rcc.hpp>

// soc
#include <soc/m4/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>

namespace {

using namespace cml;
using namespace soc::m4;
using namespace soc::m4::stm32l4;

template<typename Config_t>
uint32_t get_pll_register_config_from_factor(const Config_t& a_config, uint32_t a_enable_flag)
{
    return (static_cast<uint32_t>(a_config.divider) |
            (rcc<mcu>::PLL_config::Output::enabled == a_config.output ? a_enable_flag : 0));
}

template<typename Config_t>
uint32_t get_pllsai1_register_config_from_factor(const Config_t& a_config, uint32_t a_enable_flag)
{
    return (static_cast<uint32_t>(a_config.divider) |
            (rcc<mcu>::PLLSAI1_config::Output::enabled == a_config.output ? a_enable_flag : 0));
}

} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace cml;
using namespace cml::utils;

void rcc<mcu>::enable_clock(Clock a_clock, MSI_frequency a_freq)
{
    cml_assert(Clock::msi == a_clock);

    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);

    bit_flag::set(&(RCC->CR), RCC_CR_MSIRANGE, static_cast<uint32_t>(a_freq) << RCC_CR_MSIRANGE_Pos);

    bit_flag::set(&(RCC->CR), RCC_CR_MSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);

    bit_flag::clear(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);
    bit_flag::set(&(RCC->CR), RCC_CR_MSIRGSEL);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);
}

void rcc<mcu>::enable_clock(Clock a_clock, HSI_frequency)
{
    cml_assert(Clock::hsi == a_clock);

    bit_flag::set(&(RCC->CR), RCC_CR_HSION, RCC_CR_HSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, false);
}

void rcc<mcu>::enable_clock(Clock a_clock, LSI_frequency)
{
    cml_assert(Clock::lsi == a_clock);

    bit_flag::set(&(RCC->CSR), RCC_CSR_LSION, RCC_CSR_LSION);

    wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, false);
}

void rcc<mcu>::enable_clock(Clock a_clock, HSI48_frequency)
{
    cml_assert(Clock::hsi48 == a_clock);

    bit_flag::set(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);

    wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, false);
}

void rcc<mcu>::enable_clock(Clock a_clock, PLL_source a_source, PLLM a_m, const PLL_config& a_config)
{
    cml_assert(Clock::pll == a_clock);

    cml_assert(various::get_enum_incorrect_value<PLL_config::R::Divider>() != a_config.r.divider);
    cml_assert(various::get_enum_incorrect_value<PLL_config::Output>() != a_config.r.output);

    cml_assert(various::get_enum_incorrect_value<PLL_config::Q::Divider>() != a_config.q.divider);
    cml_assert(various::get_enum_incorrect_value<PLL_config::Output>() != a_config.q.output);

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    cml_assert(various::get_enum_incorrect_value<PLL_config::P::Divider>() != a_config.p.divider);
    cml_assert(various::get_enum_incorrect_value<PLL_config::Output>() != a_config.p.output);
#endif

    cml_assert((true == is_clock_enabled(Clock::msi) && a_source == PLL_source::msi) ||
               (true == is_clock_enabled(Clock::hsi) && a_source == PLL_source::hsi));
    cml_assert(a_config.n >= 8 && a_config.n <= 86);

    disable_clock(Clock::pll);

    RCC->PLLCFGR = static_cast<uint32_t>(a_source) | static_cast<uint32_t>(a_m) | (a_config.n << RCC_PLLCFGR_PLLN_Pos) |
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
void rcc<mcu>::enable_clock(Clock a_clock,
                            PLL_source a_source,
                            PLLM a_m,
                            const PLL_config& a_pll_config,
                            const PLLSAI1_config& a_pllsai1_config)
{
    cml_assert(Clock::pll == a_clock);

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

    RCC->PLLCFGR = static_cast<uint32_t>(a_source) | static_cast<uint32_t>(a_m) |
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
        case rcc<mcu>::Clock::msi: {
            bit_flag::clear(&(RCC->CR), RCC_CR_MSION);
            wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, true);
        }
        break;
        case rcc<mcu>::Clock::hsi: {
            bit_flag::clear(&(RCC->CR), RCC_CR_HSION);
            wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, true);
        }
        break;
        case rcc<mcu>::Clock::pll: {
            bit_flag::clear(&(RCC->CR), RCC_CR_PLLON);
            wait_until::all_bits(&(RCC->CR), RCC_CR_PLLRDY, true);

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
            bit_flag::clear(&(RCC->CR), RCC_CR_PLLSAI1ON);
            wait_until::all_bits(&(RCC->CR), RCC_CR_PLLSAI1RDY, true);
#endif
        }
        break;
        case rcc<mcu>::Clock::hsi48: {
            bit_flag::clear(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);
            wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, true);
        }
        break;
        case rcc<mcu>::Clock::lsi: {
            bit_flag::clear(&(RCC->CSR), RCC_CSR_LSION);
            wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, true);
        }
        break;
    }
}

void rcc<mcu>::set_clk48_source(CLK48_source a_source)
{
    cml_assert((a_source == CLK48_source::hsi48 && true == is_clock_enabled(Clock::hsi48)) ||
               (a_source == CLK48_source::msi && true == is_clock_enabled(Clock::msi)) ||

               ((a_source == CLK48_source::pll_q && true == is_clock_enabled(Clock::pll)) &&
                PLL_config::Output::enabled == get_pll_config().q.output)
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
               || ((a_source == CLK48_source::pll_sai1_q && is_clock_enabled(Clock::pll)) &&
                   PLLSAI1_config::Output::enabled == get_pllsai1_config().q.output)
#endif
    );

    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, static_cast<uint32_t>(a_source));
}

void rcc<mcu>::set_sysclk_source(SYSCLK_source a_source, const Bus_prescalers& a_prescalers)
{
    cml_assert(a_prescalers.ahb != various::get_enum_incorrect_value<Bus_prescalers::AHB>());
    cml_assert(a_prescalers.apb1 != various::get_enum_incorrect_value<Bus_prescalers::APB1>());
    cml_assert(a_prescalers.apb2 != various::get_enum_incorrect_value<Bus_prescalers::APB2>());

    if (false == bit_flag::is(RCC->APB1ENR1, RCC_APB1ENR1_PWREN))
    {
        bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_PWREN);
    }

    if (false == bit_flag::is(RCC->AHB1ENR, RCC_AHB1ENR_FLASHEN))
    {
        bit_flag::set(&(RCC->AHB1ENR), RCC_AHB1ENR_FLASHEN);
    }

    uint32_t frequency_hz = 0;

    switch (a_source)
    {
        case SYSCLK_source::hsi: {
            cml_assert(true == is_clock_enabled(Clock::hsi));
            frequency_hz = get_clock_frequency_hz(Clock::hsi);
        }
        break;

        case SYSCLK_source::msi: {
            cml_assert(true == is_clock_enabled(Clock::msi));
            frequency_hz = get_clock_frequency_hz(Clock::msi);
        }
        break;

        case SYSCLK_source::pll: {
            cml_assert(true == is_clock_enabled(Clock::pll));

            frequency_hz = calculate_pll_r_output_frequency();
        }
        break;
    }

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, static_cast<uint32_t>(a_source));
    wait_until::all_bits(&(RCC->CFGR), static_cast<uint32_t>(a_source) << RCC_CFGR_SWS_Pos, false);

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<uint32_t>(a_prescalers.ahb));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<uint32_t>(a_prescalers.apb1));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<uint32_t>(a_prescalers.apb2));

    SystemCoreClock = frequency_hz;

    if (SYSCLK_source::pll == a_source)
    {
        bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW_PLL);
        wait_until::all_bits(&(RCC->CFGR), RCC_CFGR_SW_PLL, false);
    }
}

rcc<mcu>::Bus_prescalers rcc<mcu>::get_bus_prescalers()
{
    return { static_cast<Bus_prescalers::AHB>(bit_flag::get(RCC->CFGR, RCC_CFGR_HPRE)),
             static_cast<Bus_prescalers::APB1>(bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE1)),
             static_cast<Bus_prescalers::APB2>(bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE2)) };
}

rcc<mcu>::PLL_config rcc<mcu>::get_pll_config()
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

rcc<mcu>::PLLSAI1_config rcc<mcu>::get_pllsai1_config()
{
    return { bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N),
             { static_cast<PLLSAI1_config::R::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R)),
               static_cast<PLLSAI1_config::Output>(bit_flag::is(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1REN)) },
             { static_cast<PLLSAI1_config::Q::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q)),
               static_cast<PLLSAI1_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLSAI1CFGR_PLLSAI1QEN)) },
             { static_cast<PLLSAI1_config::P::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P)),
               static_cast<PLLSAI1_config::Output>(bit_flag::is(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PEN)) } };
}

uint32_t rcc<mcu>::get_clock_frequency_hz(Clock a_clock)
{
    static constexpr uint32_t msi_frequency_hz_lut[] { 100u * 1000u,   200u * 1000u,   400u * 1000u,   800u * 1000u,
                                                       1u * 1000000u,  2u * 1000000u,  4u * 1000000u,  8u * 1000000u,
                                                       16u * 1000000u, 24u * 1000000u, 32u * 1000000u, 48u * 1000000u };

    switch (a_clock)
    {
        case Clock::msi: {
            return msi_frequency_hz_lut[bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos];
        }
        break;
        case rcc<mcu>::Clock::hsi: {
            return 16u * 1000000u;
        }
        break;
        case rcc<mcu>::Clock::pll: {
            return calculate_pll_r_output_frequency();
        }
        break;
        case rcc<mcu>::Clock::hsi48: {
            return 48 * 1000000u;
        }
        break;
        case rcc<mcu>::Clock::lsi: {
            return 32u * 1000u;
        }
        break;
    }

    return 0;
}

uint32_t rcc<mcu>::get_clk48_frequency_hz()
{
    CLK48_source source = get_clk48_source();

    switch (source)
    {
        case CLK48_source::hsi48: {
            return get_clock_frequency_hz(Clock::hsi48);
        }
        break;
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case CLK48_source::pll_sai1_q: {
            return calculate_pllsai1_q_output_frequency();
        }
        break;
#endif
        case rcc::CLK48_source::pll_q: {
            return calculate_pll_q_output_frequency();
        }
        break;

        case rcc::CLK48_source::msi: {
            return get_clock_frequency_hz(Clock::msi);
        }
        break;
    }

    return 0;
}

uint32_t rcc<mcu>::calculate_pll_r_output_frequency()
{
    const uint32_t m =
        (static_cast<uint32_t>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const uint32_t n = (static_cast<uint32_t>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN)) >> RCC_PLLCFGR_PLLN_Pos);

    uint32_t pllvco = 0;

    switch (static_cast<PLL_source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case PLL_source::msi: {
            pllvco = (get_clock_frequency_hz(Clock::msi) / m) * n;
        }
        break;

        case PLL_source::hsi: {
            pllvco = (get_clock_frequency_hz(Clock::hsi) / m) * n;
        }
        break;
    }

    cml_assert(pllvco >= 96 * 1000000u && pllvco <= 344 * 1000000u);

    uint32_t pllr = ((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1u) * 2u;
    return pllvco / pllr;
}

uint32_t rcc<mcu>::calculate_pll_q_output_frequency()
{
    const uint32_t m = (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const uint32_t n = bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
    const uint32_t q = ((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos) * 2u) + 2u;

    switch (static_cast<PLL_source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case PLL_source::msi: {
            return ((get_clock_frequency_hz(Clock::msi) / m) * n) / q;
        }

        case PLL_source::hsi: {
            return ((get_clock_frequency_hz(Clock::hsi) / m) * n) / q;
        }
    }

    return 0;
}

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
uint32_t rcc<mcu>::calculate_pllsai1_q_output_frequency()
{
    const uint32_t m = (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const uint32_t n = (bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos) + 1u;
    const uint32_t q =
        ((bit_flag::get(RCC->PLLCFGR, RCC_PLLSAI1CFGR_PLLSAI1Q) >> RCC_PLLSAI1CFGR_PLLSAI1Q_Pos) * 2u) + 2u;

    switch (static_cast<PLL_source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case PLL_source::msi: {
            return ((get_clock_frequency_hz(Clock::msi) / m) * n) / q;
        }

        case PLL_source::hsi: {
            return ((get_clock_frequency_hz(Clock::hsi) / m) * n) / q;
        }
    }

    return 0;
}
#endif

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4