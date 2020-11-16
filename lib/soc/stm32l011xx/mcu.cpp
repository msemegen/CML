/*
    Name: mcu.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assert.hpp>
#include <cml/frequency.hpp>

namespace {

using namespace cml;
using namespace soc::stm32l011xx;

constexpr frequency msi_frequency_lut[] { 65536,           131072,          262144,         524288,
                                          kHz_to_Hz(1048), kHz_to_Hz(2097), kHz_to_Hz(4194) };

mcu::Sysclk_frequency_change_callback pre_sysclk_frequency_change_callback;
mcu::Sysclk_frequency_change_callback post_sysclk_frequency_change_callback;

} // namespace

namespace soc {
namespace stm32l011xx {

using namespace cml;

void mcu::enable_msi_clock(Msi_frequency a_freq)
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);
    set_flag(&(RCC->ICSCR), RCC_ICSCR_MSIRANGE, static_cast<uint32_t>(a_freq) << RCC_ICSCR_MSIRANGE_Pos);
    set_flag(&(RCC->CR), RCC_CR_MSION);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY))
        ;

    clear_flag(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY))
        ;
}

void mcu::enable_hsi_clock(Hsi_frequency)
{
    set_flag(&(RCC->CR), RCC_CR_HSION);

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY))
        ;
}

void mcu::enable_lsi_clock(Lsi_frequency)
{
    set_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (false == is_flag(RCC->CSR, RCC_CSR_LSIRDY))
        ;
}

void mcu::disable_msi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    while (true == is_flag(RCC->CR, RCC_CR_MSIRDY))
        ;
}

void mcu::disable_hsi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_HSION);

    while (true == is_flag(RCC->CR, RCC_CR_HSIRDY))
        ;
}

void mcu::disable_lsi_clock()
{
    clear_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (true == is_flag(RCC->CSR, RCC_CSR_LSIRDY))
        ;
}

void mcu::enable_pll(const Pll_config& a_config)
{
    assert(true == is_clock_enabled(Clock::hsi));
    assert(Pll_config::Divider::unknown != a_config.divider);
    assert(Pll_config::Multiplier::unknown != a_config.multiplier);

    disable_pll();

    if (true == a_config.hsidiv_enabled)
    {
        set_flag(&(RCC->CR), RCC_CR_HSIDIVEN);
    }
    else
    {
        clear_flag(&(RCC->CR), RCC_CR_HSIDIVEN);
    }

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY))
        ;

    set_flag(&(RCC->CFGR),
             RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV,
             static_cast<uint32_t>(a_config.source) | static_cast<uint32_t>(a_config.divider) |
                 static_cast<uint32_t>(a_config.multiplier));

    set_flag(&(RCC->CR), RCC_CR_PLLON);

    while (false == is_flag(RCC->CR, RCC_CR_PLLRDY))
        ;
}

void mcu::disable_pll()
{
    clear_flag(&(RCC->CR), RCC_CR_PLLON);
    while (true == is_flag(RCC->CR, RCC_CR_PLLRDY))
        ;
}

void mcu::set_sysclk(Sysclk_source a_source, const Bus_prescalers& a_prescalers)
{
    if (nullptr != pre_sysclk_frequency_change_callback.function)
    {
        pre_sysclk_frequency_change_callback.function(pre_sysclk_frequency_change_callback.p_user_data);
    }

    if (false == is_flag(RCC->APB1ENR, RCC_APB1ENR_PWREN))
    {
        set_flag(&(RCC->APB1ENR), RCC_APB1ENR_PWREN);
    }

    uint32_t frequency_hz = 0;

    switch (a_source)
    {
        case Sysclk_source::msi: {
            assert(true == is_clock_enabled(Clock::msi));

            frequency_hz = msi_frequency_lut[get_flag(RCC->ICSCR, RCC_ICSCR_MSIRANGE) >> RCC_ICSCR_MSIRANGE_Pos];
        }
        break;

        case mcu::Sysclk_source::hsi: {
            assert(true == is_clock_enabled(Clock::hsi));

            frequency_hz = get_hsi_frequency_hz();
        }
        break;

        case mcu::Sysclk_source::pll: {
            assert(true == is_clock_enabled(Clock::pll));

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

    set_flag(&(FLASH->ACR), FLASH_ACR_PRFTEN | FLASH_ACR_PRE_READ);
    clear_flag(&(FLASH->ACR), FLASH_ACR_DISAB_BUF);

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
    return { static_cast<Bus_prescalers::AHB>(get_flag(RCC->CFGR, RCC_CFGR_HPRE)),
             static_cast<Bus_prescalers::APB1>(get_flag(RCC->CFGR, RCC_CFGR_PPRE1)),
             static_cast<Bus_prescalers::APB2>(get_flag(RCC->CFGR, RCC_CFGR_PPRE2)) };
}

mcu::Pll_config mcu::get_pll_config()
{
    return

        { static_cast<Pll_config::Source>(get_flag(RCC->CFGR, RCC_CFGR_PLLSRC)),
          is_flag(RCC->CR, RCC_CR_HSIDIVEN),
          static_cast<Pll_config::Multiplier>(get_flag(RCC->CFGR, RCC_CFGR_PLLMUL)),
          static_cast<Pll_config::Divider>(get_flag(RCC->CFGR, RCC_CFGR_PLLDIV)) };
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
        }
        break;

        case Voltage_scaling::_2: {
            if (a_syclk_freq <= MHz_to_Hz(8))
            {
                return Flash_latency::_0;
            }

            if (a_syclk_freq <= MHz_to_Hz(16))
            {
                return Flash_latency::_1;
            }
        }
        break;

        case Voltage_scaling::_3: {
            if (a_syclk_freq <= kHz_to_Hz(4200))
            {
                return Flash_latency::_0;
            }
        }
        break;

        case Voltage_scaling::unknown: {
            assert(a_voltage_scaling != Voltage_scaling::unknown);
        }
        break;
    }

    return Flash_latency::unknown;
}

mcu::Voltage_scaling mcu::select_voltage_scaling(Sysclk_source a_source, uint32_t a_sysclk_freq)
{
    if (Sysclk_source::msi == a_source || (Sysclk_source::pll == a_source && a_sysclk_freq <= MHz_to_Hz(4)))
    {
        return Voltage_scaling::_3;
    }

    if (Sysclk_source::hsi == a_source || (Sysclk_source::pll == a_source && a_sysclk_freq <= MHz_to_Hz(16)))
    {
        return Voltage_scaling::_2;
    }

    if (Sysclk_source::pll == a_source && a_sysclk_freq <= MHz_to_Hz(32))
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
            clear_flag(&(FLASH->ACR), FLASH_ACR_LATENCY);
        }
        break;

        case Flash_latency::_1: {
            set_flag(&(FLASH->ACR), FLASH_ACR_LATENCY);
        }
        break;

        case Flash_latency::unknown: {
            assert(Flash_latency::unknown != a_latency);
        }
        break;
    }
}

void mcu::set_voltage_scaling(Voltage_scaling a_scaling)
{
    assert(a_scaling != Voltage_scaling::unknown);

    set_flag(&(PWR->CR), PWR_CR_VOS, static_cast<uint32_t>(a_scaling));
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

void mcu::increase_sysclk_frequency(Sysclk_source a_source,
                                    frequency a_frequency_hz,
                                    const Bus_prescalers& a_prescalers)
{
    auto new_voltage_scaling = select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = select_flash_latency(a_frequency_hz, new_voltage_scaling);

    assert(Voltage_scaling::unknown != new_voltage_scaling);
    assert(Flash_latency::unknown != new_flash_latency);

    auto current_voltage_scaling = get_voltage_scaling();
    auto current_flash_latency   = get_flash_latency();

    if ((Voltage_scaling::_3 == current_voltage_scaling && Voltage_scaling::_2 == new_voltage_scaling) ||
        (Voltage_scaling::_2 == current_voltage_scaling && Voltage_scaling::_1 == new_voltage_scaling))
    {
        set_voltage_scaling(new_voltage_scaling);
        while (true == is_bit_on(PWR->CSR, PWR_CSR_VOSF_Pos))
            ;
    }

    if (Flash_latency::_1 == current_flash_latency && Flash_latency::_0 == new_flash_latency)
    {
        set_flash_latency(new_flash_latency);
        while (true == is_bit_on(FLASH->ACR, FLASH_ACR_LATENCY_Pos))
            ;
    }

    set_sysclk_source(a_source);
    set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

void mcu::decrease_sysclk_frequency(Sysclk_source a_source,
                                    frequency a_frequency_hz,
                                    const Bus_prescalers& a_prescalers)
{
    set_sysclk_source(a_source);

    auto new_voltage_scaling = select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = select_flash_latency(a_frequency_hz, new_voltage_scaling);

    assert(Voltage_scaling::unknown != new_voltage_scaling);
    assert(Flash_latency::unknown != new_flash_latency);

    auto current_voltage_scaling = get_voltage_scaling();
    auto current_flash_latency   = get_flash_latency();

    if ((Voltage_scaling::_2 == current_voltage_scaling && Voltage_scaling::_3 == new_voltage_scaling) ||
        (Voltage_scaling::_1 == current_voltage_scaling && Voltage_scaling::_2 == new_voltage_scaling))
    {
        set_voltage_scaling(new_voltage_scaling);
        while (true == is_bit_on(PWR->CSR, PWR_CSR_VOSF_Pos))
            ;
    }

    if (Flash_latency::_0 == current_flash_latency && Flash_latency::_1 == new_flash_latency)
    {
        set_flash_latency(new_flash_latency);
        while (false == is_bit_on(FLASH->ACR, FLASH_ACR_LATENCY_Pos))
            ;
    }

    set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

uint32_t mcu::calculate_frequency_from_pll_configuration()
{
    constexpr uint32_t m_lut[] = { 3, 4, 6, 8, 12, 16, 24, 32, 48 };
    constexpr uint32_t d_lut[] = { 2, 3, 4 };

    const uint32_t mi = (get_flag(RCC->CFGR, RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_Pos) - 0;
    const uint32_t di = (get_flag(RCC->CFGR, RCC_CFGR_PLLDIV) >> RCC_CFGR_PLLDIV_Pos) - 1;

    assert(di < sizeof(d_lut) / sizeof(d_lut[0]));
    assert(mi < sizeof(m_lut) / sizeof(m_lut[0]));

    return get_hsi_frequency_hz() / (true == is_flag(RCC->CR, RCC_CR_HSIDIVEN) ? 4 : 1) * m_lut[mi] / d_lut[di];
}

} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx