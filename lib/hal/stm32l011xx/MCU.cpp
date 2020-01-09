/*
    Name: mcu.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

//this
#include <hal/stm32l011xx/mcu.hpp>

//cml
#include <common/assert.hpp>
#include <common/bit.hpp>
#include <common/frequency.hpp>
#include <common/macros.hpp>

namespace {

using namespace cml::common;

constexpr frequency msi_frequency_lut[]
{
    Hz(65536),
    Hz(131072),
    Hz(262144),
    Hz(524288),
    kHz(1048),
    kHz(2097),
    kHz(4194)
};

} // namespace ::

namespace cml {
namespace hal {
namespace stm32l011xx {

using namespace cml::common;

void MCU::enable_MSI_clock(MSI_frequency a_freq)
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);
    set_flag(&(RCC->ICSCR), RCC_ICSCR_MSIRANGE, static_cast<uint32>(a_freq) << RCC_ICSCR_MSIRANGE_Pos);
    set_flag(&(RCC->CR), RCC_CR_MSION);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));

    clear_flag(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));
}

void MCU::enable_HSI_clock(HSI_frequency a_freq)
{
    unused(a_freq);

    set_flag(&(RCC->CR), RCC_CR_HSION);

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY));
}

void MCU::enable_LSI_clock(LSI_frequency a_freq)
{
    unused(a_freq);

    set_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (false == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
}

void MCU::disable_MSI_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    while (true == is_flag(RCC->CR, RCC_CR_MSIRDY));
}

void MCU::disable_HSI_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_HSION);

    while (true == is_flag(RCC->CR, RCC_CR_HSIRDY));
}

void MCU::disable_LSI_clock()
{
    clear_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (true == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
}

void MCU::enable_PLL(PLL_clock_source a_source, const PLL_config& a_pll_config)
{
    unused(a_source);
    assert(true == this->is_clock_enabled(Clock::HSI));
    assert(PLL_config::Divider::unknown != a_pll_config.divider);
    assert(PLL_config::Multiplier::unknown != a_pll_config.multiplier);

    if (true == a_pll_config.hsidiv_enabled)
    {
        set_flag(&(RCC->CR), RCC_CR_HSIDIVEN);
    }
    else
    {
        clear_flag(&(RCC->CR), RCC_CR_HSIDIVEN);
    }

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY));

    set_flag(&(RCC->CFGR), RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV,
                           static_cast<uint32>(a_source)             |
                           static_cast<uint32>(a_pll_config.divider) |
                           static_cast<uint32>(a_pll_config.multiplier));

    set_flag(&(RCC->CR), RCC_CR_PLLON);

    while (false == is_flag(RCC->CR, RCC_CR_PLLRDY));
}

void MCU::disable_PLL()
{
    clear_flag(&(RCC->CR), RCC_CR_PLLON);
    while (true == is_flag(RCC->CR, RCC_CR_PLLRDY));
}

void MCU::set_SYSCLK(SYSCLK_source a_source, const Bus_prescalers& a_prescalers)
{
    if (nullptr != this->pre_SYSCLK_frequency_change_callback.p_function)
    {
        this->pre_SYSCLK_frequency_change_callback.p_function(this->pre_SYSCLK_frequency_change_callback.a_p_user_data);
    }

    if (false == is_flag(RCC->APB1ENR, RCC_APB1ENR_PWREN))
    {
        set_flag(&(RCC->APB1ENR), RCC_APB1ENR_PWREN);
    }

    switch (a_source)
    {
        case SYSCLK_source::MSI:
        {
            assert(true == this->is_clock_enabled(Clock::MSI));

            uint32 i = get_flag(RCC->ICSCR, RCC_ICSCR_MSIRANGE) >> RCC_ICSCR_MSIRANGE_Pos;
            uint32 frequency_hz = msi_frequency_lut[i];

            if (SystemCoreClock > frequency_hz)
            {
                this->decrease_SYSCLK_frequency(SYSCLK_source::MSI, frequency_hz, a_prescalers);
            }
            else if (SystemCoreClock < frequency_hz)
            {
                this->increase_SYSCLK_frequency(SYSCLK_source::MSI, frequency_hz, a_prescalers);
            }
        }
        break;

        case MCU::SYSCLK_source::HSI:
        {
            assert(true == this->is_clock_enabled(Clock::HSI));

            if (SystemCoreClock > config::clock::HSI_FREQUENCY_HZ)
            {
                this->decrease_SYSCLK_frequency(SYSCLK_source::HSI,
                                                config::clock::HSI_FREQUENCY_HZ,
                                                a_prescalers);
            }
            else if (SystemCoreClock < config::clock::HSI_FREQUENCY_HZ)
            {
                this->increase_SYSCLK_frequency(SYSCLK_source::HSI,
                                                config::clock::HSI_FREQUENCY_HZ,
                                                a_prescalers);
            }
        }
        break;

        case MCU::SYSCLK_source::PLL:
        {
            assert(true == this->is_clock_enabled(Clock::PLL));

            uint32 frequency_hz = this->calculate_frequency_from_PLL_configuration();

            if (SystemCoreClock > frequency_hz)
            {
                this->decrease_SYSCLK_frequency(SYSCLK_source::PLL, frequency_hz, a_prescalers);
            }
            else if (SystemCoreClock < frequency_hz)
            {
                this->increase_SYSCLK_frequency(SYSCLK_source::PLL, frequency_hz, a_prescalers);
            }
        }
        break;
    }

    set_flag(&(FLASH->ACR), FLASH_ACR_PRFTEN | FLASH_ACR_PRE_READ);
    clear_flag(&(FLASH->ACR), FLASH_ACR_DISAB_BUF);

    if (nullptr != this->post_SYSCLK_frequency_change_callback.p_function)
    {
        this->post_SYSCLK_frequency_change_callback.p_function(this->post_SYSCLK_frequency_change_callback.a_p_user_data);
    }
}

MCU::Flash_latency MCU::select_flash_latency(uint32 a_syclk_freq, Voltage_scaling a_voltage_scaling)
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
        }
        break;

        case Voltage_scaling::_2:
        {
            if (a_syclk_freq <= MHz(8))
            {
                return Flash_latency::_0;
            }

            if (a_syclk_freq <= MHz(16))
            {
                return Flash_latency::_1;
            }
        }
        break;

        case Voltage_scaling::_3:
        {
            if (a_syclk_freq <= kHz(4200))
            {
                return Flash_latency::_0;
            }
        }
        break;

        case Voltage_scaling::unknown:
        {
            assert(a_voltage_scaling != Voltage_scaling::unknown);
        }
        break;
    }

    return Flash_latency::unknown;
}

MCU::Voltage_scaling MCU::select_voltage_scaling(SYSCLK_source a_source, uint32 a_sysclk_freq)
{
    if (SYSCLK_source::MSI == a_source || (SYSCLK_source::PLL == a_source && a_sysclk_freq <= MHz(4)))
    {
        return Voltage_scaling::_3;
    }

    if (SYSCLK_source::HSI == a_source || (SYSCLK_source::PLL == a_source && a_sysclk_freq <= MHz(16)))
    {
        return Voltage_scaling::_2;
    }

    if (SYSCLK_source::PLL == a_source && a_sysclk_freq <= MHz(32))
    {
        return Voltage_scaling::_1;
    }

    return Voltage_scaling::unknown;
}

void MCU::set_flash_latency(Flash_latency a_latency)
{
    switch (a_latency)
    {
        case Flash_latency::_0:
        {
            clear_flag(&(FLASH->ACR), FLASH_ACR_LATENCY);
        }
        break;

        case Flash_latency::_1:
        {
            set_flag(&(FLASH->ACR), FLASH_ACR_LATENCY);
        }
        break;

        case Flash_latency::unknown:
        {
            assert(Flash_latency::unknown != a_latency);
        }
        break;
    }
}

void MCU::set_voltage_scaling(Voltage_scaling a_scaling)
{
    assert(a_scaling != Voltage_scaling::unknown);

    set_flag(&(PWR->CR), PWR_CR_VOS, static_cast<uint32>(a_scaling));
}

void MCU::set_SYSCLK_source(SYSCLK_source a_sysclk_source)
{
    set_flag(&(RCC->CFGR), RCC_CFGR_SW, static_cast<uint32>(a_sysclk_source));
    while (false == is_flag(RCC->CFGR, static_cast<uint32>(a_sysclk_source) << RCC_CFGR_SWS_Pos));
}

void MCU::set_bus_prescalers(const Bus_prescalers& a_prescalers)
{
    assert(Bus_prescalers::AHB::unknown  != a_prescalers.ahb);
    assert(Bus_prescalers::APB1::unknown != a_prescalers.apb1);
    assert(Bus_prescalers::APB2::unknown != a_prescalers.apb2);

    set_flag(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<uint32>(a_prescalers.ahb));
    set_flag(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<uint32>(a_prescalers.apb1));
    set_flag(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<uint32>(a_prescalers.apb2));
}

void MCU::increase_SYSCLK_frequency(SYSCLK_source a_source,
                                    uint32 a_frequency_hz,
                                    const Bus_prescalers& a_prescalers)
{
    auto new_voltage_scaling = this->select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = this->select_flash_latency(a_frequency_hz, new_voltage_scaling);

    assert(Voltage_scaling::unknown != new_voltage_scaling);
    assert(Flash_latency::unknown   != new_flash_latency);

    auto current_voltage_scaling = this->get_voltage_scaling();
    auto current_flash_latency   = this->get_flash_latency();

    if ((Voltage_scaling::_3 == current_voltage_scaling && Voltage_scaling::_2 == new_voltage_scaling) ||
        (Voltage_scaling::_2 == current_voltage_scaling && Voltage_scaling::_1 == new_voltage_scaling))
    {
        this->set_voltage_scaling(new_voltage_scaling);
        while (true == get_bit(PWR->CSR, PWR_CSR_VOSF_Pos));
    }

    if (Flash_latency::_1 == current_flash_latency && Flash_latency::_0 == new_flash_latency)
    {
        this->set_flash_latency(new_flash_latency);
        while (true == get_bit(FLASH->ACR, FLASH_ACR_LATENCY_Pos));
    }

    this->set_SYSCLK_source(a_source);
    this->set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

void MCU::decrease_SYSCLK_frequency(SYSCLK_source a_source,
                                    uint32 a_frequency_hz,
                                    const Bus_prescalers& a_prescalers)
{
    this->set_SYSCLK_source(a_source);

    auto new_voltage_scaling = this->select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = this->select_flash_latency(a_frequency_hz, new_voltage_scaling);

    assert(Voltage_scaling::unknown != new_voltage_scaling);
    assert(Flash_latency::unknown   != new_flash_latency);

    auto current_voltage_scaling = this->get_voltage_scaling();
    auto current_flash_latency   = this->get_flash_latency();

    if ((Voltage_scaling::_2 == current_voltage_scaling && Voltage_scaling::_3 == new_voltage_scaling) ||
        (Voltage_scaling::_1 == current_voltage_scaling && Voltage_scaling::_2 == new_voltage_scaling))
    {
        this->set_voltage_scaling(new_voltage_scaling);
        while (true == get_bit(PWR->CSR, PWR_CSR_VOSF_Pos));
    }

    if (Flash_latency::_0 == current_flash_latency && Flash_latency::_1 == new_flash_latency)
    {
        this->set_flash_latency(new_flash_latency);
        while (false == get_bit(FLASH->ACR, FLASH_ACR_LATENCY_Pos));
    }

    this->set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

uint32 MCU::calculate_frequency_from_PLL_configuration()
{
    constexpr uint32 m_lut[] = { 3, 4, 6, 8, 12, 16, 24, 32, 48 };
    constexpr uint32 d_lut[] = { 2, 3, 4 };

    const uint32 mi = (get_flag(RCC->CFGR, RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_Pos) - 0;
    const uint32 di = (get_flag(RCC->CFGR, RCC_CFGR_PLLDIV) >> RCC_CFGR_PLLDIV_Pos) - 1;

    assert(di < sizeof(d_lut) / sizeof(d_lut[0]));
    assert(mi < sizeof(m_lut) / sizeof(m_lut[0]));

    return config::clock::HSI_FREQUENCY_HZ / (true == is_flag(RCC->CR, RCC_CR_HSIDIVEN) ? 4 : 1) * m_lut[mi] / d_lut[di];
}

} // namespace stm32l011xx
} // namespace hal
} // namespace cml

#endif // STM32L011xx