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
#include <common/handy.hpp>

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

void c_mcu::enable_msi_clock(e_msi_frequency a_freq)
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    clear_flag(&(RCC->ICSCR), RCC_ICSCR_MSIRANGE);
    set_flag(&(RCC->ICSCR), static_cast<uint32>(a_freq) << RCC_ICSCR_MSIRANGE_Pos);

    set_flag(&(RCC->CR), RCC_CR_MSION);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));

    clear_flag(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));
    set_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::msi));
}

void c_mcu::enable_hsi_clock(e_hsi_frequency a_freq)
{
    _unused(a_freq);

    clear_flag(&(RCC->CR), RCC_CR_HSION);
    set_flag(&(RCC->CR), RCC_CR_HSION);

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY));
    set_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::hsi));
}

void c_mcu::enable_lsi_clock(e_lsi_frequency a_freq)
{
    _unused(a_freq);

    clear_flag(&(RCC->CSR), RCC_CSR_LSION);
    set_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (false == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
    set_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::lsi));
}

void c_mcu::disable_msi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    while (true == is_flag(RCC->CR, RCC_CR_MSIRDY));
    clear_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::msi));
}

void c_mcu::disable_hsi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_HSION);

    while (true == is_flag(RCC->CR, RCC_CR_HSIRDY));
    clear_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::hsi));
}

void c_mcu::disable_lsi_clock()
{
    clear_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (true == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
    clear_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::lsi));
}

void c_mcu::enable_pll(e_pll_clock_source a_source, const s_pll_config& a_pll_config)
{
    _unused(a_source);
    _assert(true == this->is_clock_enabled(e_clock::hsi));
    _assert(s_pll_config::e_pll_divider::unknown != a_pll_config.pll_divider);
    _assert(s_pll_config::e_pll_multiplier::unknown != a_pll_config.pll_multiplier);

    if (true == a_pll_config.hsi_divader)
    {
        set_flag(&(RCC->CR), RCC_CR_HSIDIVEN);
    }
    else
    {
        clear_flag(&(RCC->CR), RCC_CR_HSIDIVEN);
    }

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY));

    clear_flag(&(RCC->CFGR), RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV);
    set_flag(&(RCC->CFGR), RCC_CFGR_PLLSRC_HSI                           |
                           static_cast<uint32>(a_pll_config.pll_divider) |
                           static_cast<uint32>(a_pll_config.pll_multiplier));

    set_flag(&(RCC->CR), RCC_CR_PLLON);

    while (false == is_flag(RCC->CR, RCC_CR_PLLRDY));
    set_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::pll));
}

void c_mcu::disable_pll()
{
    clear_flag(&(RCC->CR), RCC_CR_PLLON);
    while (true == is_flag(RCC->CR, RCC_CR_PLLRDY));
}

void c_mcu::set_sysclk(e_sysclk_source a_source, const s_bus_prescalers& a_prescalers)
{
    if (false == is_flag(RCC->APB1ENR, RCC_APB1ENR_PWREN))
    {
        set_flag(&(RCC->APB1ENR), RCC_APB1ENR_PWREN);
    }

    switch (a_source)
    {
        case e_sysclk_source::msi:
        {
            _assert(true == this->is_clock_enabled(e_clock::msi));

            uint32 i = get_flag(RCC->ICSCR, RCC_ICSCR_MSIRANGE) >> RCC_ICSCR_MSIRANGE_Pos;
            uint32 msi_frequency_hz = msi_frequency_lut[i];

            if (SystemCoreClock > msi_frequency_hz)
            {
                this->decrease_sysclk_frequency(e_sysclk_source::msi, msi_frequency_hz, a_prescalers);
            }
            else if (SystemCoreClock < msi_frequency_hz)
            {
                this->increase_sysclk_frequency(e_sysclk_source::msi, msi_frequency_hz, a_prescalers);
            }
        }
        break;

        case c_mcu::e_sysclk_source::hsi:
        {
            _assert(true == this->is_clock_enabled(e_clock::hsi));

            if (SystemCoreClock > s_config::s_clock::hsi_frequency_hz)
            {
                this->decrease_sysclk_frequency(e_sysclk_source::hsi,
                                                s_config::s_clock::hsi_frequency_hz,
                                                a_prescalers);
            }
            else if (SystemCoreClock < s_config::s_clock::hsi_frequency_hz)
            {
                this->increase_sysclk_frequency(e_sysclk_source::msi,
                                                s_config::s_clock::hsi_frequency_hz,
                                                a_prescalers);
            }
        }
        break;

        case c_mcu::e_sysclk_source::pll:
        {
            _assert(true == this->is_clock_enabled(e_clock::pll));

            uint32 pll_frequency_hz = this->calculate_frequency_from_pll_configuration();

            if (SystemCoreClock > pll_frequency_hz)
            {
                this->decrease_sysclk_frequency(e_sysclk_source::hsi, pll_frequency_hz, a_prescalers);
            }
            else if (SystemCoreClock < pll_frequency_hz)
            {
                this->increase_sysclk_frequency(e_sysclk_source::msi, pll_frequency_hz, a_prescalers);
            }
        }
        break;
    }
}

bool c_mcu::enable_low_power_run()
{
    bool ret = SystemCoreClock < msi_frequency_lut[1];

    if (true == ret)
    {
        set_flag(&(PWR->CR), PWR_CR_LPRUN);
    }

    return ret;
}
void c_mcu::disable_low_power_run()
{
    clear_flag(&(PWR->CR), PWR_CR_LPRUN);
}

c_mcu::e_flash_latency c_mcu::select_flash_latency(uint32 a_syclk_freq, e_voltage_scaling a_voltage_scaling)
{
    switch (a_voltage_scaling)
    {
        case e_voltage_scaling::_1:
        {
            if (a_syclk_freq <= MHz(16))
            {
                return e_flash_latency::_0;
            }
            else if (a_syclk_freq <= MHz(32))
            {
                return e_flash_latency::_1;
            }
        }
        break;

        case e_voltage_scaling::_2:
        {
            if (a_syclk_freq <= MHz(8))
            {
                return e_flash_latency::_0;
            }
            else if (a_syclk_freq <= MHz(16))
            {
                return e_flash_latency::_1;
            }
        }
        break;

        case e_voltage_scaling::_3:
        {
            if (a_syclk_freq <= kHz(4200))
            {
                return e_flash_latency::_0;
            }
        }
        break;
    }

    return e_flash_latency::uknown;
}

c_mcu::e_voltage_scaling c_mcu::select_voltage_scaling(uint32 a_sysclk_freq)
{
    if (a_sysclk_freq <= kHz(4200))
    {
        return e_voltage_scaling::_3;
    }
    else if (a_sysclk_freq <= MHz(8))
    {
        return e_voltage_scaling::_2;
    }

    return e_voltage_scaling::_1;
}

void c_mcu::set_flash_latency(e_flash_latency a_latency)
{
    switch (a_latency)
    {
        case e_flash_latency::_0:
        {
            clear_flag(&(FLASH->ACR), FLASH_ACR_LATENCY);
        }
        break;

        case e_flash_latency::_1:
        {
            set_flag(&(FLASH->ACR), FLASH_ACR_LATENCY);
        }
        break;

        case e_flash_latency::uknown:
        {
            _assert(false);
        }
        break;
    }
}

void c_mcu::set_voltage_scaling(e_voltage_scaling a_scaling)
{
    uint32 pwr_cr = PWR->CR;

    clear_flag(&pwr_cr, PWR_CR_VOS);
    set_flag(&pwr_cr, static_cast<uint32>(a_scaling));

    PWR->CR = pwr_cr;
}

void c_mcu::increase_sysclk_frequency(e_sysclk_source a_source,
                                      uint32 a_frequency_hz,
                                      const s_bus_prescalers& a_prescalers)
{

}

void c_mcu::decrease_sysclk_frequency(e_sysclk_source a_source,
                                      uint32 a_frequency_hz,
                                      const s_bus_prescalers& a_prescalers)
{

}

uint32 c_mcu::calculate_frequency_from_pll_configuration()
{

}

} // namespace stm32l011xx
} // namespace hal
} // namespace cml

#endif // STM32L011xx