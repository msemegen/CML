/*
    Name: mcu.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/mcu.hpp>

// cml
#include <cml/debug/assert.hpp>
#include <cml/frequency.hpp>
#include <cml/utils/wait.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l452xx;

constexpr frequency msi_frequency_lut[] { kHz_to_Hz(100u), kHz_to_Hz(200u), kHz_to_Hz(400u), kHz_to_Hz(800u),
                                          MHz_to_Hz(1u),   MHz_to_Hz(2u),   MHz_to_Hz(4u),   MHz_to_Hz(8u),
                                          MHz_to_Hz(16u),  MHz_to_Hz(24u),  MHz_to_Hz(32u),  MHz_to_Hz(48u) };

mcu::Sysclk_frequency_change_callback pre_sysclk_frequency_change_callback;
mcu::Sysclk_frequency_change_callback post_sysclk_frequency_change_callback;

template<typename Config_t>
uint32_t get_pll_register_config_from_factor(const Config_t& a_config, uint32_t a_enable_flag)
{
    return (Config_t::Divider::unknown != a_config.divider ?
                (static_cast<uint32_t>(a_config.divider) | (true == a_config.output_enabled ? a_enable_flag : 0)) :
                0);
}

} // namespace

namespace soc {
namespace stm32l452xx {

using namespace cml;
using namespace cml::utils;

void mcu::enable_msi_clock(Msi_frequency a_freq)
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    set_flag(&(RCC->CR), RCC_CR_MSIRANGE, static_cast<uint32_t>(a_freq) << RCC_CR_MSIRANGE_Pos);

    set_flag(&(RCC->CR), RCC_CR_MSION);

    wait::until(&(RCC->CR), RCC_CR_MSIRDY, false);

    clear_flag(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);
    set_flag(&(RCC->CR), RCC_CR_MSIRGSEL);

    wait::until(&(RCC->CR), RCC_CR_MSIRDY, false);
}

void mcu::enable_hsi_clock(Hsi_frequency)
{
    set_flag(&(RCC->CR), RCC_CR_HSION, RCC_CR_HSION);

    wait::until(&(RCC->CR), RCC_CR_HSIRDY, false);
}

void mcu::enable_lsi_clock(Lsi_frequency)
{
    set_flag(&(RCC->CSR), RCC_CSR_LSION, RCC_CSR_LSION);

    wait::until(&(RCC->CSR), RCC_CSR_LSIRDY, false);
}

void mcu::enable_hsi48_clock(Hsi48_frequency)
{
    set_flag(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);

    wait::until(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, false);
}

void mcu::disable_msi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    wait::until(&(RCC->CR), RCC_CR_MSIRDY, true);
}

void mcu::disable_hsi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_HSION);

    wait::until(&(RCC->CR), RCC_CR_HSIRDY, true);
}

void mcu::disable_lsi_clock()
{
    clear_flag(&(RCC->CSR), RCC_CSR_LSION);

    wait::until(&(RCC->CSR), RCC_CSR_LSIRDY, true);
}

void mcu::disable_hsi48_clock()
{
    clear_flag(&(RCC->CRRCR), RCC_CRRCR_HSI48ON);

    wait::until(&(RCC->CRRCR), RCC_CRRCR_HSI48ON, true);
}

void mcu::enable_pll(const Pll_config& a_config)
{
    assert((true == is_clock_enabled(Clock::msi) && a_config.source == Pll_config::Source::msi) ||
           (true == is_clock_enabled(Clock::hsi) && a_config.source == Pll_config::Source::hsi));
    assert((a_config.pll.n >= 8 && a_config.pll.n <= 86) && (a_config.pllsai1.n >= 8 && a_config.pllsai1.n <= 86));

    disable_pll();

    RCC->PLLCFGR = static_cast<uint32_t>(a_config.source) | static_cast<uint32_t>(a_config.m) |
                   (a_config.pll.n << RCC_PLLCFGR_PLLN_Pos) |
                   get_pll_register_config_from_factor(a_config.pll.p, RCC_PLLCFGR_PLLPEN) |
                   get_pll_register_config_from_factor(a_config.pll.q, RCC_PLLCFGR_PLLQEN) |
                   get_pll_register_config_from_factor(a_config.pll.r, RCC_PLLCFGR_PLLREN);

    set_flag(&(RCC->CR), RCC_CR_PLLON);
    while (false == get_flag(RCC->CR, RCC_CR_PLLRDY))
        ;

    RCC->PLLSAI1CFGR = (a_config.pllsai1.n << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) |
                       get_pll_register_config_from_factor(a_config.pllsai1.p, RCC_PLLSAI1CFGR_PLLSAI1PEN) |
                       get_pll_register_config_from_factor(a_config.pllsai1.q, RCC_PLLSAI1CFGR_PLLSAI1QEN) |
                       get_pll_register_config_from_factor(a_config.pllsai1.r, RCC_PLLSAI1CFGR_PLLSAI1REN);

    set_flag(&(RCC->CR), RCC_CR_PLLSAI1ON);
    while (false == get_flag(RCC->CR, RCC_CR_PLLSAI1RDY))
        ;
}

void mcu::disable_pll()
{
    clear_flag(&(RCC->CR), RCC_CR_PLLON);
    while (true == is_flag(RCC->CR, RCC_CR_PLLRDY))
        ;

    clear_flag(&(RCC->CR), RCC_CR_PLLSAI1ON);
    while (true == is_flag(RCC->CR, RCC_CR_PLLSAI1RDY))
        ;
}

void mcu::set_clk48_clock_mux_source(Clk48_mux_source a_source)
{
    assert((a_source == Clk48_mux_source::hsi48 && is_clock_enabled(Clock::hsi48)) ||
           (a_source == Clk48_mux_source::msi && is_clock_enabled(Clock::msi)) ||

           ((a_source == Clk48_mux_source::pll_q && is_clock_enabled(Clock::pll)) &&
            true == get_pll_config().pll.q.output_enabled) ||
           ((a_source == Clk48_mux_source::pll_sai1_q && is_clock_enabled(Clock::pll)) &&
            true == get_pll_config().pllsai1.q.output_enabled));

    assert(get_clk48_mux_freqency_hz() <= MHz_to_Hz(48));

    set_flag(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, static_cast<uint32_t>(a_source));
}

void mcu::set_sysclk(Sysclk_source a_source, const Bus_prescalers& a_prescalers)
{
    if (nullptr != pre_sysclk_frequency_change_callback.function)
    {
        pre_sysclk_frequency_change_callback.function(pre_sysclk_frequency_change_callback.p_user_data);
    }

    if (false == is_flag(RCC->APB1ENR1, RCC_APB1ENR1_PWREN))
    {
        set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_PWREN);
    }

    if (false == is_flag(RCC->AHB1ENR, RCC_AHB1ENR_FLASHEN))
    {
        set_flag(&(RCC->AHB1ENR), RCC_AHB1ENR_FLASHEN);
    }

    uint32_t frequency_hz = 0;

    switch (a_source)
    {
        case Sysclk_source::hsi: {
            assert(true == is_clock_enabled(Clock::hsi));

            frequency_hz = get_hsi_frequency_hz();
        }
        break;

        case Sysclk_source::msi: {
            assert(true == is_clock_enabled(Clock::msi));

            frequency_hz = msi_frequency_lut[get_flag(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos];
        }
        break;

        case Sysclk_source::pll: {
            assert(true == is_clock_enabled(Clock::pll));

            frequency_hz = calculate_pll_r_output_frequency();
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
        while (false == is_flag(RCC->CFGR, RCC_CFGR_SW_PLL))
            ;
    }

    if (Flash_latency::_0 != get_flash_latency())
    {
        set_flag(&(FLASH->ACR), FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_ICEN);
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

void mcu::enable_interrupt_line(Interrupt_line a_line, uint32_t a_priority)
{
    NVIC_SetPriority(static_cast<IRQn_Type>(a_line), a_priority);
    NVIC_EnableIRQ(static_cast<IRQn_Type>(a_line));
}

void mcu::disable_interrupt_line(Interrupt_line a_line)
{
    NVIC_DisableIRQ(static_cast<IRQn_Type>(a_line));
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
    return { static_cast<Bus_prescalers::AHB>(get_flag(RCC->CFGR, RCC_CFGR_HPRE)),
             static_cast<Bus_prescalers::APB1>(get_flag(RCC->CFGR, RCC_CFGR_PPRE1)),
             static_cast<Bus_prescalers::APB2>(get_flag(RCC->CFGR, RCC_CFGR_PPRE2)) };
}

mcu::Pll_config mcu::get_pll_config()
{
    return

        { static_cast<Pll_config::Source>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)),
          static_cast<Pll_config::M>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)),
          { get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLN),
            { static_cast<Pll_config::PLL::R::Divider>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLR)),
              is_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN) },
            { static_cast<Pll_config::PLL::Q::Divider>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ)),
              is_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN) },
            { static_cast<Pll_config::PLL::P::Divider>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLP)),
              is_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLPEN) } },

          { get_flag(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N),
            { static_cast<Pll_config::PLLSAI1::R::Divider>(get_flag(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R)),
              is_flag(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1REN) },
            { static_cast<Pll_config::PLLSAI1::Q::Divider>(get_flag(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q)),
              is_flag(RCC->PLLCFGR, RCC_PLLSAI1CFGR_PLLSAI1QEN) },
            { static_cast<Pll_config::PLLSAI1::P::Divider>(get_flag(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1P)),
              is_flag(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1PEN) } } };
}

uint32_t mcu::get_clk48_mux_freqency_hz()
{
    Clk48_mux_source source = get_clk48_mux_source();

    switch (source)
    {
        case Clk48_mux_source::hsi48: {
            return MHz_to_Hz(48);
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
            uint32_t msi_range = get_flag(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            return msi_frequency_lut[msi_range];
        }
        break;
    }

    return 0;
}

mcu::Flash_latency mcu::select_flash_latency(uint32_t a_syclk_freq, Voltage_scaling a_voltage_scaling)
{
    switch (a_voltage_scaling)
    {
        case Voltage_scaling::_1: {
            if (a_syclk_freq <= MHz_to_Hz(16))
            {
                return Flash_latency::_0;
            }

            if (a_syclk_freq <= MHz_to_Hz(32))
            {
                return Flash_latency::_1;
            }

            if (a_syclk_freq <= MHz_to_Hz(48))
            {
                return Flash_latency::_2;
            }

            if (a_syclk_freq <= MHz_to_Hz(64))
            {
                return Flash_latency::_3;
            }

            if (a_syclk_freq <= MHz_to_Hz(80))
            {
                return Flash_latency::_4;
            }
        }
        break;

        case Voltage_scaling::_2: {
            if (a_syclk_freq <= MHz_to_Hz(6))
            {
                return Flash_latency::_0;
            }

            if (a_syclk_freq <= MHz_to_Hz(12))
            {
                return Flash_latency::_1;
            }

            if (a_syclk_freq <= MHz_to_Hz(18))
            {
                return Flash_latency::_2;
            }

            if (a_syclk_freq <= MHz_to_Hz(26))
            {
                return Flash_latency::_3;
            }
        }
        break;

        case Voltage_scaling::unkown: {
            assert(Voltage_scaling::unkown != a_voltage_scaling);
        }
        break;
    }

    return Flash_latency::unknown;
}

mcu::Voltage_scaling mcu::select_voltage_scaling(Sysclk_source a_source, uint32_t a_sysclk_freq)
{
    if ((Sysclk_source::msi == a_source && a_sysclk_freq <= MHz_to_Hz(24)) ||
        (Sysclk_source::pll == a_source && a_sysclk_freq <= MHz_to_Hz(26)))
    {
        return Voltage_scaling::_2;
    }

    if ((Sysclk_source::msi == a_source && a_sysclk_freq <= MHz_to_Hz(48)) ||
        (Sysclk_source::pll == a_source && a_sysclk_freq <= MHz_to_Hz(80)) || (Sysclk_source::hsi == a_source))
    {
        return Voltage_scaling::_1;
    }

    return Voltage_scaling::unkown;
}

void mcu::set_flash_latency(Flash_latency a_latency)
{
    assert(a_latency != Flash_latency::unknown);

    set_flag(&(FLASH->ACR), FLASH_ACR_LATENCY, static_cast<uint32_t>(a_latency));

    while (false == is_flag(FLASH->ACR, static_cast<uint32_t>(a_latency)))
        ;
}

void mcu::set_voltage_scaling(Voltage_scaling a_scaling)
{
    assert(a_scaling != Voltage_scaling::unkown);

    set_flag(&(PWR->CR1), PWR_CR1_VOS, static_cast<uint32_t>(a_scaling));

    while (false == is_bit_on(PWR->SR2, PWR_SR2_VOSF_Pos))
        ;
}

void mcu::set_sysclk_source(Sysclk_source a_sysclk_source)
{
    set_flag(&(RCC->CFGR), RCC_CFGR_SW, static_cast<uint32_t>(a_sysclk_source));
    while (false == is_flag(RCC->CFGR, static_cast<uint32_t>(a_sysclk_source) << RCC_CFGR_SWS_Pos))
        ;
}

void mcu::set_bus_prescalers(const Bus_prescalers& a_prescalers)
{
    assert(Bus_prescalers::AHB::unknown != a_prescalers.ahb);
    assert(Bus_prescalers::APB1::unknown != a_prescalers.apb1);
    assert(Bus_prescalers::APB2::unknown != a_prescalers.apb2);

    set_flag(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<uint32_t>(a_prescalers.ahb));
    set_flag(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<uint32_t>(a_prescalers.apb1));
    set_flag(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<uint32_t>(a_prescalers.apb2));
}

void mcu::increase_sysclk_frequency(Sysclk_source a_source, uint32_t a_frequency_hz, const Bus_prescalers& a_prescalers)
{
    auto new_voltage_scaling = select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = select_flash_latency(a_frequency_hz, new_voltage_scaling);

    assert(Voltage_scaling::unkown != new_voltage_scaling);
    assert(Flash_latency::unknown != new_flash_latency);

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

void mcu::decrease_sysclk_frequency(Sysclk_source a_source, uint32_t a_frequency_hz, const Bus_prescalers& a_prescalers)
{
    set_sysclk_source(a_source);

    auto new_voltage_scaling = select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = select_flash_latency(a_frequency_hz, new_voltage_scaling);

    assert(Voltage_scaling::unkown != new_voltage_scaling);
    assert(Flash_latency::unknown != new_flash_latency);

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

uint32_t mcu::calculate_pll_r_output_frequency()
{
    const uint32_t m = (static_cast<uint32_t>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const uint32_t n = (static_cast<uint32_t>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLN)) >> RCC_PLLCFGR_PLLN_Pos);

    uint32_t pllvco = 0;

    switch (static_cast<Pll_config::Source>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case Pll_config::Source::msi: {
            uint32_t msi_range = get_flag(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            pllvco             = (msi_frequency_lut[msi_range] / m) * n;
        }
        break;

        case Pll_config::Source::hsi: {
            pllvco = (get_hsi_frequency_hz() / m) * n;
        }
        break;

        case Pll_config::Source::unknown: {
            assert(false);
        }
        break;
    }

    assert(pllvco >= MHz_to_Hz(96) && pllvco <= MHz_to_Hz(344));

    uint32_t pllr = ((get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1u) * 2u;
    return pllvco / pllr;
}

uint32_t mcu::calculate_pll_q_output_frequency()
{
    const uint32_t m = (get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const uint32_t n = get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
    const uint32_t q = ((get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos) * 2u) + 2u;

    switch (static_cast<Pll_config::Source>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case Pll_config::Source::msi: {
            const uint32_t msi_range = get_flag(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            return ((msi_frequency_lut[msi_range] / m) * n) / q;
        }

        case Pll_config::Source::hsi: {
            return ((get_hsi_frequency_hz() / m) * n) / q;
        }

        case Pll_config::Source::unknown: {
            assert(false);
            return 0;
        }
    }

    return 0;
}

uint32_t mcu::calculate_pllsai1_q_output_frequency()
{
    const uint32_t m = (get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const uint32_t n = (get_flag(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1N) >> RCC_PLLSAI1CFGR_PLLSAI1N_Pos) + 1u;
    const uint32_t q = ((get_flag(RCC->PLLCFGR, RCC_PLLSAI1CFGR_PLLSAI1Q) >> RCC_PLLSAI1CFGR_PLLSAI1Q_Pos) * 2u) + 2u;

    switch (static_cast<Pll_config::Source>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case Pll_config::Source::msi: {
            uint32_t msi_range = get_flag(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            return ((msi_frequency_lut[msi_range] / m) * n) / q;
        }

        case Pll_config::Source::hsi: {
            return ((get_hsi_frequency_hz() / m) * n) / q;
        }

        default: {
            assert(false);
            return 0;
        }
    }

    return 0;
}

} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx