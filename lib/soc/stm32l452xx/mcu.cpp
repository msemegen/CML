/*
 *   Name: mcu.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l452xx;

constexpr uint32_t msi_frequency_hz_lut[] { 100u * 1000u,   200u * 1000u,   400u * 1000u,   800u * 1000u,
                                            1u * 1000000u,  2u * 1000000u,  4u * 1000000u,  8u * 1000000u,
                                            16u * 1000000u, 24u * 1000000u, 32u * 1000000u, 48u * 1000000u };

mcu::Sysclk_frequency_change_callback pre_sysclk_frequency_change_callback;
mcu::Sysclk_frequency_change_callback post_sysclk_frequency_change_callback;

template<typename Config_t>
uint32_t get_pll_register_config_from_factor(const Config_t& a_config, uint32_t a_enable_flag)
{
    return (Config_t::Divider::unknown != a_config.divider ?
                (static_cast<uint32_t>(a_config.divider) |
                 (mcu::Pll_config::Output::enabled == a_config.output ? a_enable_flag : 0)) :
                0);
}

} // namespace

namespace soc {
namespace stm32l452xx {

using namespace cml;
using namespace cml::utils;

void mcu::enable_msi_clock(Msi_frequency a_freq)
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);

    bit_flag::set(&(RCC->CR), RCC_CR_MSIRANGE, static_cast<uint32_t>(a_freq) << RCC_CR_MSIRANGE_Pos);

    bit_flag::set(&(RCC->CR), RCC_CR_MSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);

    bit_flag::clear(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);
    bit_flag::set(&(RCC->CR), RCC_CR_MSIRGSEL);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, false);
}

void mcu::enable_hsi_clock(Hsi_frequency)
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSION, RCC_CR_HSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, false);
}

void mcu::enable_lsi_clock(Lsi_frequency)
{
    bit_flag::set(&(RCC->CSR), RCC_CSR_LSION, RCC_CSR_LSION);

    wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, false);
}

void mcu::enable_hsi48_clock(Hsi48_frequency)
{
    bit_flag::set(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);

    wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, false);
}

void mcu::disable_msi_clock()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_MSIRDY, true);
}

void mcu::disable_hsi_clock()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_HSION);

    wait_until::all_bits(&(RCC->CR), RCC_CR_HSIRDY, true);
}

void mcu::disable_lsi_clock()
{
    bit_flag::clear(&(RCC->CSR), RCC_CSR_LSION);

    wait_until::all_bits(&(RCC->CSR), RCC_CSR_LSIRDY, true);
}

void mcu::disable_hsi48_clock()
{
    bit_flag::clear(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);

    wait_until::all_bits(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, true);
}

void mcu::enable_pll(const Pll_config& a_config)
{
    cml_assert((true == is_clock_enabled(Clock::msi) && a_config.source == Pll_config::Source::msi) ||
               (true == is_clock_enabled(Clock::hsi) && a_config.source == Pll_config::Source::hsi));
    cml_assert((a_config.pll.n >= 8 && a_config.pll.n <= 86) && (a_config.pllsai1.n >= 8 && a_config.pllsai1.n <= 86));

    disable_pll();

    RCC->PLLCFGR = static_cast<uint32_t>(a_config.source) | static_cast<uint32_t>(a_config.m) |
                   (a_config.pll.n << RCC_PLLCFGR_PLLN_Pos) |
                   get_pll_register_config_from_factor(a_config.pll.p, RCC_PLLCFGR_PLLPEN) |
                   get_pll_register_config_from_factor(a_config.pll.q, RCC_PLLCFGR_PLLQEN) |
                   get_pll_register_config_from_factor(a_config.pll.r, RCC_PLLCFGR_PLLREN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLON);
    while (false == bit_flag::get(RCC->CR, RCC_CR_PLLRDY))
        ;

    RCC->PLLSAI1CFGR = (a_config.pllsai1.n << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                       get_pll_register_config_from_factor(a_config.pllsai1.p, RCC_PLLSAI1CFGR_PLLSAI1PEN) |
                       get_pll_register_config_from_factor(a_config.pllsai1.q, RCC_PLLSAI1CFGR_PLLSAI1QEN) |
                       get_pll_register_config_from_factor(a_config.pllsai1.r, RCC_PLLSAI1CFGR_PLLSAI1REN);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLSAI1ON);
    while (false == bit_flag::get(RCC->CR, RCC_CR_PLLSAI1RDY))
        ;
}

void mcu::disable_pll()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_PLLON);
    while (true == bit_flag::is(RCC->CR, RCC_CR_PLLRDY))
        ;

    bit_flag::clear(&(RCC->CR), RCC_CR_PLLSAI1ON);
    while (true == bit_flag::is(RCC->CR, RCC_CR_PLLSAI1RDY))
        ;
}

void mcu::set_clk48_clock_mux_source(Clk48_mux_source a_source)
{
    cml_assert((a_source == Clk48_mux_source::hsi48 && is_clock_enabled(Clock::hsi48)) ||
               (a_source == Clk48_mux_source::msi && is_clock_enabled(Clock::msi)) ||

               ((a_source == Clk48_mux_source::pll_q && is_clock_enabled(Clock::pll)) &&
                Pll_config::Output::enabled == get_pll_config().pll.q.output) ||
               ((a_source == Clk48_mux_source::pll_sai1_q && is_clock_enabled(Clock::pll)) &&
                Pll_config::Output::enabled == get_pll_config().pllsai1.q.output));

    cml_assert(get_clk48_mux_freqency_hz() <= 48 * 1000000u);

    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, static_cast<uint32_t>(a_source));
}

void mcu::set_sysclk(Sysclk_source a_source, const Bus_prescalers& a_prescalers, Voltage_scaling a_voltage_scaling)
{
    if (nullptr != pre_sysclk_frequency_change_callback.function)
    {
        pre_sysclk_frequency_change_callback.function(pre_sysclk_frequency_change_callback.p_user_data);
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

    if (SystemCoreClock > frequency_hz)
    {
        decrease_sysclk_frequency(a_source, frequency_hz, a_prescalers, a_voltage_scaling);
    }
    else if (SystemCoreClock < frequency_hz)
    {
        increase_sysclk_frequency(a_source, frequency_hz, a_prescalers, a_voltage_scaling);
    }

    if (Sysclk_source::pll == a_source)
    {
        bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW_PLL);
        while (false == bit_flag::is(RCC->CFGR, RCC_CFGR_SW_PLL))
            ;
    }

    if (internal_flash::Latency::_0 != internal_flash::get_latency())
    {
        bit_flag::set(&(FLASH->ACR), FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_ICEN);
    }

    if (nullptr != post_sysclk_frequency_change_callback.function)
    {
        post_sysclk_frequency_change_callback.function(post_sysclk_frequency_change_callback.p_user_data);
    }
}

void mcu::set_nvic(const NVIC_config& a_config)
{
    NVIC_SetPriorityGrouping(static_cast<uint32_t>(a_config.grouping));
    __set_BASEPRI(a_config.base_priority);
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
    return {
        static_cast<Pll_config::Source>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)),
        static_cast<Pll_config::M>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)),
        { bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN),
          { static_cast<Pll_config::PLL::R::Divider>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLR)),
            static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN)) },
          { static_cast<Pll_config::PLL::Q::Divider>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ)),
            static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN)) },
          { static_cast<Pll_config::PLL::P::Divider>(bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLP)),
            static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN)) } },

        { bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N),
          { static_cast<Pll_config::PLLSAI1::R::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R)),
            static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1REN)) },
          { static_cast<Pll_config::PLLSAI1::Q::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q)),
            static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLCFGR, RCC_PLLSAI1CFGR_PLLSAI1QEN)) },
          { static_cast<Pll_config::PLLSAI1::P::Divider>(bit_flag::get(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P)),
            static_cast<Pll_config::Output>(bit_flag::is(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PEN)) } }
    };
}

uint32_t mcu::get_clk48_mux_freqency_hz()
{
    Clk48_mux_source source = get_clk48_mux_source();

    switch (source)
    {
        case Clk48_mux_source::hsi48: {
            return 48 * 1000000u;
        }
        break;

        case Clk48_mux_source::pll_sai1_q: {
            return calculate_pllsai1_q_output_frequency();
        }
        break;

        case mcu::Clk48_mux_source::pll_q: {
            calculate_pll_q_output_frequency();
        }
        break;

        case mcu::Clk48_mux_source::msi: {
            uint32_t msi_range = bit_flag::get(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            return msi_frequency_hz_lut[msi_range];
        }
        break;
    }

    return 0;
}

internal_flash::Latency mcu::select_flash_latency(uint32_t a_syclk_freq, Voltage_scaling a_voltage_scaling)
{
    switch (a_voltage_scaling)
    {
        case Voltage_scaling::_1: {
            if (a_syclk_freq <= 16u * 1000000u)
            {
                return internal_flash::Latency::_0;
            }

            if (a_syclk_freq <= 32u * 1000000u)
            {
                return internal_flash::Latency::_1;
            }

            if (a_syclk_freq <= 48u * 1000000u)
            {
                return internal_flash::Latency::_2;
            }

            if (a_syclk_freq <= 64u * 1000000u)
            {
                return internal_flash::Latency::_3;
            }

            if (a_syclk_freq <= 80u * 1000000u)
            {
                return internal_flash::Latency::_4;
            }
        }
        break;

        case Voltage_scaling::_2: {
            if (a_syclk_freq <= 6u * 1000000u)
            {
                return internal_flash::Latency::_0;
            }

            if (a_syclk_freq <= 12u * 1000000u)
            {
                return internal_flash::Latency::_1;
            }

            if (a_syclk_freq <= 18u * 1000000u)
            {
                return internal_flash::Latency::_2;
            }

            if (a_syclk_freq <= 26u * 1000000u)
            {
                return internal_flash::Latency::_3;
            }
        }
        break;
    }

    return Flash_latency::unknown;
}

mcu::Voltage_scaling mcu::select_voltage_scaling(Sysclk_source a_source, uint32_t a_sysclk_freq)
{
    if ((Sysclk_source::msi == a_source && a_sysclk_freq <= 24 * 1000000u) ||
        (Sysclk_source::pll == a_source && a_sysclk_freq <= 26 * 1000000u))
    {
        return Voltage_scaling::_2;
    }

    if ((Sysclk_source::msi == a_source && a_sysclk_freq <= 48 * 1000000u) ||
        (Sysclk_source::pll == a_source && a_sysclk_freq <= 80 * 1000000u) || (Sysclk_source::hsi == a_source))
    {
        return Voltage_scaling::_1;
    }

    return Voltage_scaling::unkown;
}

void mcu::set_flash_latency(Flash_latency a_latency)
{
    cml_assert(a_latency != Flash_latency::unknown);

    bit_flag::set(&(FLASH->ACR), FLASH_ACR_LATENCY, static_cast<uint32_t>(a_latency));

    while (false == bit_flag::is(FLASH->ACR, static_cast<uint32_t>(a_latency)))
        ;
}

void mcu::set_voltage_scaling(Voltage_scaling a_scaling)
{
    bit_flag::set(&(PWR->CR1), PWR_CR1_VOS, static_cast<uint32_t>(a_scaling));

    while (true == bit::is(PWR->SR2, PWR_SR2_VOSF_Pos))
        ;
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

void mcu::increase_sysclk_frequency(Sysclk_source a_source,
                                    uint32_t a_frequency_hz,
                                    const Bus_prescalers& a_prescalers,
                                    Voltage_scaling a_voltage_scaling)
{
    cml_assert(a_voltage_scaling == Voltage_scaling::_2 &&
               ((get_sysclk_source() == Sysclk_source::msi && a_frequency_hz <= 24u * 1000000u) ||
                (get_sysclk_source() == Sysclk_source::pll && a_frequency_hz <= 26 * 1000000u)));

    cml_assert(Voltage_scaling::unkown != new_voltage_scaling);
    cml_assert(Flash_latency::unknown != new_flash_latency);

    if (a_voltage_scaling != get_voltage_scaling())
    {
        set_voltage_scaling(a_voltage_scaling);
    }

    if (new_flash_latency != internal_flash::get_latency())
    {
        set_flash_latency(new_flash_latency);
    }

    set_sysclk_source(a_source);
    set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

void mcu::decrease_sysclk_frequency(Sysclk_source a_source,
                                    uint32_t a_frequency_hz,
                                    const Bus_prescalers& a_prescalers,
                                    Voltage_scaling a_voltage_scaling)
{
    cml_assert(a_voltage_scaling == Voltage_scaling::_2 &&
               ((get_sysclk_source() == Sysclk_source::msi && a_frequency_hz <= 24u * 1000000u) ||
                (get_sysclk_source() == Sysclk_source::pll && a_frequency_hz <= 26 * 1000000u)));

    set_sysclk_source(a_source);

    auto new_voltage_scaling = select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = select_flash_latency(a_frequency_hz, new_voltage_scaling);

    cml_assert(Voltage_scaling::unkown != new_voltage_scaling);
    cml_assert(Flash_latency::unknown != new_flash_latency);

    if (new_flash_latency != internal_flash::get_latency())
    {
        set_flash_latency(new_flash_latency);
    }

    if (a_voltage_scaling != get_voltage_scaling())
    {
        set_voltage_scaling(a_voltage_scaling);
    }

    set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

uint32_t mcu::calculate_pll_r_output_frequency()
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

        case Pll_config::Source::unknown: {
            cml_assert(false);
        }
        break;
    }

    cml_assert(pllvco >= 96 * 1000000u && pllvco <= 344 * 1000000u);

    uint32_t pllr = ((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1u) * 2u;
    return pllvco / pllr;
}

uint32_t mcu::calculate_pll_q_output_frequency()
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

        case Pll_config::Source::unknown: {
            cml_assert(false);
            return 0;
        }
    }

    return 0;
}

uint32_t mcu::calculate_pllsai1_q_output_frequency()
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

        default: {
            cml_assert(false);
            return 0;
        }
    }

    return 0;
}

} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx