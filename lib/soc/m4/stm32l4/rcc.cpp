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

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

constexpr uint32_t msi_frequency_hz_lut[] { 100u * 1000u,   200u * 1000u,   400u * 1000u,   800u * 1000u,
                                            1u * 1000000u,  2u * 1000000u,  4u * 1000000u,  8u * 1000000u,
                                            16u * 1000000u, 24u * 1000000u, 32u * 1000000u, 48u * 1000000u };

template<typename Config_t>
uint32_t get_pll_register_config_from_factor(const Config_t& a_config, uint32_t a_enable_flag)
{
    return (static_cast<uint32_t>(a_config.divider) |
            (rcc<mcu>::Pll_config::Output::enabled == a_config.output ? a_enable_flag : 0));
}

#endif

} // namespace

namespace soc {
namespace m4 {
namespace stm32l4 {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

using namespace cml;
using namespace cml::utils;

void rcc<mcu>::enable_msi_clock(Msi_frequency a_freq)
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);

    bit_flag::set(&(RCC->CR), RCC_CR_MSIRANGE, static_cast<uint32_t>(a_freq) << RCC_CR_MSIRANGE_Pos);

    bit_flag::set(&(RCC->CR), RCC_CR_MSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);

    bit_flag::clear(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);
    bit_flag::set(&(RCC->CR), RCC_CR_MSIRGSEL);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);
}

void rcc<mcu>::enable_hsi_clock(Hsi_frequency)
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSION, RCC_CR_HSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, false);
}

void rcc<mcu>::enable_lsi_clock(Lsi_frequency)
{
    bit_flag::set(&(RCC->CSR), RCC_CSR_LSION, RCC_CSR_LSION);

    wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, false);
}

void rcc<mcu>::enable_hsi48_clock(Hsi48_frequency)
{
    bit_flag::set(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);

    wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, false);
}

void rcc<mcu>::disable_msi_clock()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, true);
}

void rcc<mcu>::disable_hsi_clock()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_HSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, true);
}

void rcc<mcu>::disable_lsi_clock()
{
    bit_flag::clear(&(RCC->CSR), RCC_CSR_LSION);

    wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, true);
}

void rcc<mcu>::disable_hsi48_clock()
{
    bit_flag::clear(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);

    wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, true);
}

void rcc<mcu>::enable_pll(const Pll_config& a_config)
{
    cml_assert(various::get_enum_incorrect_value<Pll_config::Source>() != a_config.source);
    cml_assert(various::get_enum_incorrect_value<Pll_config::M>() != a_config.m);

    cml_assert(various::get_enum_incorrect_value<Pll_config::PLL::R::Divider>() != a_config.pll.r.divider);
    cml_assert(various::get_enum_incorrect_value<Pll_config::Output>() != a_config.pll.r.output);

    cml_assert(various::get_enum_incorrect_value<Pll_config::PLL::Q::Divider>() != a_config.pll.q.divider);
    cml_assert(various::get_enum_incorrect_value<Pll_config::Output>() != a_config.pll.q.output);

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    cml_assert(various::get_enum_incorrect_value<Pll_config::PLL::P::Divider>() != a_config.pll.p.divider);
    cml_assert(various::get_enum_incorrect_value<Pll_config::Output>() != a_config.pll.p.output);
#endif

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    cml_assert(various::get_enum_incorrect_value<Pll_config::PLLSAI1::R::Divider>() != a_config.pllsai1.r.divider);
    cml_assert(various::get_enum_incorrect_value<Pll_config::Output>() != a_config.pllsai1.r.output);

    cml_assert(various::get_enum_incorrect_value<Pll_config::PLLSAI1::Q::Divider>() != a_config.pllsai1.q.divider);
    cml_assert(various::get_enum_incorrect_value<Pll_config::Output>() != a_config.pllsai1.q.output);

    cml_assert(various::get_enum_incorrect_value<Pll_config::PLLSAI1::P::Divider>() != a_config.pllsai1.p.divider);
    cml_assert(various::get_enum_incorrect_value<Pll_config::Output>() != a_config.pllsai1.p.output);
#endif

    cml_assert((true == is_clock_enabled(Clock::msi) && a_config.source == Pll_config::Source::msi) ||
               (true == is_clock_enabled(Clock::hsi) && a_config.source == Pll_config::Source::hsi));
    cml_assert((a_config.pll.n >= 8 && a_config.pll.n <= 86));
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    cml_assert(a_config.pllsai1.n >= 8 && a_config.pllsai1.n <= 86);
#endif

    disable_pll();

    RCC->PLLCFGR = static_cast<uint32_t>(a_config.source) | static_cast<uint32_t>(a_config.m) |
                   (a_config.pll.n << RCC_PLLCFGR_PLLN_Pos) |
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
                   get_pll_register_config_from_factor(a_config.pll.p, RCC_PLLCFGR_PLLPEN) |
#endif
                   get_pll_register_config_from_factor(a_config.pll.q, RCC_PLLCFGR_PLLQEN) |
                   get_pll_register_config_from_factor(a_config.pll.r, RCC_PLLCFGR_PLLREN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLON);
    while (false == bit_flag::get(RCC->CR, RCC_CR_PLLRDY))
        ;
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    RCC->PLLSAI1CFGR = (a_config.pllsai1.n << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                       get_pll_register_config_from_factor(a_config.pllsai1.p, RCC_PLLSAI1CFGR_PLLSAI1PEN) |
                       get_pll_register_config_from_factor(a_config.pllsai1.q, RCC_PLLSAI1CFGR_PLLSAI1QEN) |
                       get_pll_register_config_from_factor(a_config.pllsai1.r, RCC_PLLSAI1CFGR_PLLSAI1REN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLSAI1ON);
    while (false == bit_flag::get(RCC->CR, RCC_CR_PLLSAI1RDY))
        ;
#endif
}

void rcc<mcu>::disable_pll()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_PLLON);
    while (true == bit_flag::is(RCC->CR, RCC_CR_PLLRDY))
        ;
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    bit_flag::clear(&(RCC->CR), RCC_CR_PLLSAI1ON);
    while (true == bit_flag::is(RCC->CR, RCC_CR_PLLSAI1RDY))
        ;
#endif
}

void rcc<mcu>::set_clk48_clock_mux_source(Clk48_mux_source a_source)
{
    cml_assert((a_source == Clk48_mux_source::hsi48 && is_clock_enabled(Clock::hsi48)) ||
               (a_source == Clk48_mux_source::msi && is_clock_enabled(Clock::msi)) ||

               ((a_source == Clk48_mux_source::pll_q && is_clock_enabled(Clock::pll)) &&
                Pll_config::Output::enabled == get_pll_config().pll.q.output)
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
               || ((a_source == Clk48_mux_source::pll_sai1_q && is_clock_enabled(Clock::pll)) &&
                   Pll_config::Output::enabled == get_pll_config().pllsai1.q.output)
#endif
    );

    cml_assert(get_clk48_mux_freqency_hz() <= 48 * 1000000u);

    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, static_cast<uint32_t>(a_source));
}

void rcc<mcu>::set_sysclk(Sysclk_source a_source,
                          const Bus_prescalers& a_prescalers,
                          const Sysclk_frequency_change_callback& a_pre_callback,
                          const Sysclk_frequency_change_callback& a_post_callback)
{
    if (nullptr != a_pre_callback.function)
    {
        a_pre_callback.function(a_pre_callback.p_user_data);
    }

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
        case Sysclk_source::hsi: {
            cml_assert(true == is_clock_enabled(Clock::hsi));

            frequency_hz = get_hsi_frequency_hz();
        }
        break;

        case Sysclk_source::msi: {
            cml_assert(true == is_clock_enabled(Clock::msi));

            frequency_hz = msi_frequency_hz_lut[bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos];
        }
        break;

        case Sysclk_source::pll: {
            cml_assert(true == is_clock_enabled(Clock::pll));

            frequency_hz = calculate_pll_r_output_frequency();
        }
        break;
    }

    set_sysclk_source(a_source);
    set_bus_prescalers(a_prescalers);

    SystemCoreClock = frequency_hz;

    if (Sysclk_source::pll == a_source)
    {
        bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW_PLL);
        while (false == bit_flag::is(RCC->CFGR, RCC_CFGR_SW_PLL))
            ;
    }

    if (nullptr != a_post_callback.function)
    {
        a_post_callback.function(a_post_callback.p_user_data);
    }
}

rcc<mcu>::Bus_prescalers rcc<mcu>::get_bus_prescalers()
{
    return { static_cast<Bus_prescalers::AHB>(bit_flag::get(RCC->CFGR, RCC_CFGR_HPRE)),
             static_cast<Bus_prescalers::APB1>(bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE1)),
             static_cast<Bus_prescalers::APB2>(bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE2)) };
}

rcc<mcu>::Pll_config rcc<mcu>::get_pll_config()
{
    return
    {
        static_cast<Pll_config::Source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)),
            static_cast<Pll_config::M>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)),
        {
            bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN),
                { static_cast<Pll_config::PLL::R::Divider>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLR)),
                  static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN)) },
                { static_cast<Pll_config::PLL::Q::Divider>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ)),
                  static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN)) },
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
            {
                static_cast<Pll_config::PLL::P::Divider>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLP)),
                    static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN))
            }
#endif
        }
        ,
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        {
            bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N),
                { static_cast<Pll_config::PLLSAI1::R::Divider>(
                      bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R)),
                  static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1REN)) },
                { static_cast<Pll_config::PLLSAI1::Q::Divider>(
                      bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q)),
                  static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLSAI1CFGR_PLLSAI1QEN)) },
            {
                static_cast<Pll_config::PLLSAI1::P::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P)),
                    static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PEN))
            }
        }
#endif
    };
}

uint32_t rcc<mcu>::get_clk48_mux_freqency_hz()
{
    Clk48_mux_source source = get_clk48_mux_source();

    switch (source)
    {
        case Clk48_mux_source::hsi48: {
            return 48 * 1000000u;
        }
        break;
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        case Clk48_mux_source::pll_sai1_q: {
            return calculate_pllsai1_q_output_frequency();
        }
        break;
#endif
        case rcc::Clk48_mux_source::pll_q: {
            calculate_pll_q_output_frequency();
        }
        break;

        case rcc::Clk48_mux_source::msi: {
            uint32_t msi_range = bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            return msi_frequency_hz_lut[msi_range];
        }
        break;
    }

    return 0;
}

void rcc<mcu>::set_sysclk_source(Sysclk_source a_sysclk_source)
{
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, static_cast<uint32_t>(a_sysclk_source));
    while (false == bit_flag::is(RCC->CFGR, static_cast<uint32_t>(a_sysclk_source) << RCC_CFGR_SWS_Pos))
        ;
}

void rcc<mcu>::set_bus_prescalers(const Bus_prescalers& a_prescalers)
{
    cml_assert(a_prescalers.ahb != various::get_enum_incorrect_value<Bus_prescalers::AHB>());
    cml_assert(a_prescalers.apb1 != various::get_enum_incorrect_value<Bus_prescalers::APB1>());
    cml_assert(a_prescalers.apb2 != various::get_enum_incorrect_value<Bus_prescalers::APB2>());

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<uint32_t>(a_prescalers.ahb));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<uint32_t>(a_prescalers.apb1));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<uint32_t>(a_prescalers.apb2));
}

uint32_t rcc<mcu>::calculate_pll_r_output_frequency()
{
    const uint32_t m =
        (static_cast<uint32_t>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const uint32_t n = (static_cast<uint32_t>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN)) >> RCC_PLLCFGR_PLLN_Pos);

    uint32_t pllvco = 0;

    switch (static_cast<Pll_config::Source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case Pll_config::Source::msi: {
            uint32_t msi_range = bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            pllvco             = (msi_frequency_hz_lut[msi_range] / m) * n;
        }
        break;

        case Pll_config::Source::hsi: {
            pllvco = (get_hsi_frequency_hz() / m) * n;
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

    switch (static_cast<Pll_config::Source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case Pll_config::Source::msi: {
            const uint32_t msi_range = bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            return ((msi_frequency_hz_lut[msi_range] / m) * n) / q;
        }

        case Pll_config::Source::hsi: {
            return ((get_hsi_frequency_hz() / m) * n) / q;
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

    switch (static_cast<Pll_config::Source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case Pll_config::Source::msi: {
            uint32_t msi_range = bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            return ((msi_frequency_hz_lut[msi_range] / m) * n) / q;
        }

        case Pll_config::Source::hsi: {
            return ((get_hsi_frequency_hz() / m) * n) / q;
        }
    }

    return 0;
}
#endif

#endif

} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4