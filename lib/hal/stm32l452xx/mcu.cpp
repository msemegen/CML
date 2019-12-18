/*
    Name: mcu.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L452xx

//this
#include <hal/stm32l452xx/mcu.hpp>

//external
#include <stm32l452xx.h>

//cml
#include <common/assert.hpp>
#include <common/frequency.hpp>
#include <common/handy.hpp>

namespace {

using namespace cml::common;

constexpr frequency msi_frequency_lut[]
{
    kHz(100u),
    kHz(200u),
    kHz(400u),
    kHz(800u),
    MHz(1u),
    MHz(2u),
    MHz(4u),
    MHz(8u),
    MHz(16u),
    MHz(24u),
    MHz(32u),
    MHz(48u)
};

} // namespace ::

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace common;

void c_mcu::enable_msi_clock(e_msi_frequency a_freq)
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    clear_flag(&(RCC->CR), RCC_CR_MSIRANGE);
    set_flag(&(RCC->CR), static_cast<uint32>(a_freq) << RCC_CR_MSIRANGE_Pos);

    set_flag(&(RCC->CR), RCC_CR_MSION);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));

    clear_flag(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);
    set_flag(&(RCC->CR), RCC_CR_MSIRGSEL);

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
    uint32 pllcfgr = static_cast<uint32>(a_pll_config.m_divider)    |
                     a_pll_config.n_divider << RCC_PLLCFGR_PLLN_Pos |
                     static_cast<uint32>(a_pll_config.r_divider);

    switch (a_source)
    {
        case e_pll_clock_source::msi:
        {
            _assert(true == this->is_clock_enabled(e_clock::msi));

            set_flag(&pllcfgr, RCC_PLLCFGR_PLLSRC_MSI);
        }
        break;

        case e_pll_clock_source::hsi:
        {
            _assert(true == this->is_clock_enabled(e_clock::hsi));

            set_flag(&pllcfgr, RCC_PLLCFGR_PLLSRC_HSI);
        }
        break;
    }

    clear_flag(&(RCC->PLLCFGR), RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLR);

    set_flag(&(RCC->PLLCFGR), pllcfgr);
    set_flag(&(RCC->CR), RCC_CR_PLLON);
    set_flag(&(RCC->PLLCFGR), RCC_PLLCFGR_PLLREN);

    while (false == get_flag(RCC->CR, RCC_CR_PLLRDY));
    set_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::pll));
}

void c_mcu::disable_pll()
{
    clear_flag(&(RCC->CR), RCC_CR_PLLON);

    while (true == is_flag(RCC->CR, RCC_CR_PLLRDY));
    clear_bit(&(this->enabled_clocks), static_cast<uint32>(e_clock::pll));
}

void c_mcu::set_sysclk(e_sysclk_source a_source, const s_bus_prescalers& a_prescalers, const s_nvic& a_nvic_settings)
{
    _assert(s_bus_prescalers::e_ahb::unknown != a_prescalers.ahb);
    _assert(s_bus_prescalers::e_apb1::unknown != a_prescalers.apb1);
    _assert(s_bus_prescalers::e_apb2::unknown != a_prescalers.apb2);

    if (nullptr != this->pre_sysclock_frequency_callback.p_function)
    {
        this->pre_sysclock_frequency_callback.p_function(this->pre_sysclock_frequency_callback.a_p_user_data);
    }

    clear_flag(&(RCC->CFGR), RCC_CFGR_SW_Msk);

    if (RCC_APB1ENR1_PWREN != get_flag(RCC->APB1ENR1, RCC_APB1ENR1_PWREN))
    {
        set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_PWREN);
    }

    if (RCC_AHB1ENR_FLASHEN != get_flag(RCC->AHB1ENR, RCC_AHB1ENR_FLASHEN))
    {
        set_flag(&(RCC->AHB1ENR), RCC_AHB1ENR_FLASHEN);
    }

    switch (a_source)
    {
        case e_sysclk_source::hsi:
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
                this->increase_sysclk_frequency(e_sysclk_source::hsi,
                                                s_config::s_clock::hsi_frequency_hz,
                                                a_prescalers);
            }
        }
        break;

        case e_sysclk_source::msi:
        {
            _assert(true == this->is_clock_enabled(e_clock::msi));

            uint32 i = get_flag(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
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

        case e_sysclk_source::pll:
        {
            _assert(true == this->is_clock_enabled(e_clock::pll));

            uint32 pll_frequency_hz = this->calculate_frequency_from_pll_configuration();

            if (SystemCoreClock > pll_frequency_hz)
            {
                this->decrease_sysclk_frequency(e_sysclk_source::msi, pll_frequency_hz, a_prescalers);
            }
            else if (SystemCoreClock < pll_frequency_hz)
            {
                this->increase_sysclk_frequency(e_sysclk_source::msi, pll_frequency_hz, a_prescalers);
            }

            set_flag(&(RCC->CFGR), RCC_CFGR_SW_PLL);
            while (false == is_flag(RCC->CFGR, RCC_CFGR_SW_PLL));
        }
        break;
    }

    set_flag(&(FLASH->ACR), FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_ICEN);

    if (nullptr != this->post_sysclock_frequency_callback.p_function)
    {
        this->post_sysclock_frequency_callback.p_function(this->post_sysclock_frequency_callback.a_p_user_data);
    }

    NVIC_SetPriorityGrouping(a_nvic_settings.priority_grouping);
    __set_BASEPRI(a_nvic_settings.base_priority);
}

bool c_mcu::enable_low_power_run()
{
    bool ret = SystemCoreClock < MHz(2);

    if (true == ret)
    {
        set_flag(&(PWR->CR1), PWR_CR1_LPR);
    }

    return ret;
}

void c_mcu::disable_low_power_run()
{
    clear_flag(&(PWR->CR1), PWR_CR1_LPR);
    while (true == is_flag(PWR->SR2, PWR_SR2_REGLPF));
}

c_mcu::e_flash_latency c_mcu::select_flash_latency(uint32 a_syclk_freq,
                                                   e_voltage_scaling a_voltage_scaling)
{
    //RM0394
    //DocID027295 Rev 3
    //75, 3.3.3 Read access latency

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
            else if (a_syclk_freq <= MHz(48))
            {
                return e_flash_latency::_2;
            }
            else if (a_syclk_freq <= MHz(64))
            {
                return e_flash_latency::_3;
            }
            else if (a_syclk_freq <= MHz(80))
            {
                return e_flash_latency::_4;
            }
        }
        break;

        case e_voltage_scaling::_2:
        {
            if (a_syclk_freq <= MHz(6))
            {
                return e_flash_latency::_0;
            }
            else if (a_syclk_freq <= MHz(12))
            {
                return e_flash_latency::_1;
            }
            else if (a_syclk_freq <= MHz(18))
            {
                return e_flash_latency::_2;
            }
            else if (a_syclk_freq <= MHz(26))
            {
                return e_flash_latency::_3;
            }
        }
        break;

        case e_voltage_scaling::unkown:
        {
            _assert(false);
        }
        break;
    }

    return e_flash_latency::unknown;
}

c_mcu::e_voltage_scaling c_mcu::select_voltage_scaling(uint32 a_sysclk_freq)
{
    e_voltage_scaling retval = e_voltage_scaling::unkown;

    //RM0394
    //DocID027295 Rev 3
    //130, 5.1.7 Dynamic voltage scaling management

    if (a_sysclk_freq <= MHz(26))
    {
        retval = e_voltage_scaling::_2;
    }
    else if (a_sysclk_freq <= MHz(80))
    {
        retval = e_voltage_scaling::_1;
    }

    return retval;
}

void c_mcu::set_flash_latency(e_flash_latency a_latency)
{
    _assert(a_latency != e_flash_latency::unknown);

    uint32 flash_acr = FLASH->ACR;

    clear_flag(&flash_acr, FLASH_ACR_LATENCY);
    set_flag(&flash_acr, static_cast<uint32>(a_latency));

    FLASH->ACR = flash_acr;

    while (false == is_flag(FLASH->ACR, static_cast<uint32>(a_latency)));
}

void c_mcu::set_voltage_scaling(e_voltage_scaling a_scaling)
{
    _assert(a_scaling != e_voltage_scaling::unkown);

    uint32 pwr_cr1 = PWR->CR1;

    clear_flag(&pwr_cr1, PWR_CR1_VOS);
    set_flag(&pwr_cr1, static_cast<uint32>(a_scaling));

    PWR->CR1 = pwr_cr1;
}

void c_mcu::increase_sysclk_frequency(e_sysclk_source a_source,
                                      uint32 a_frequency_hz,
                                      const s_bus_prescalers& a_prescalers)
{
    //RM0394
    //DocID027295 Rev 3
    //130, 5.1.7 Dynamic voltage scaling management
    //75, 3.3.3 Read access latency

    auto new_voltage_scaling = this->select_voltage_scaling(a_frequency_hz);
    auto new_flash_latency   = this->select_flash_latency(a_frequency_hz, new_voltage_scaling);

    _assert(e_voltage_scaling::unkown != new_voltage_scaling);
    _assert(e_flash_latency::unknown != new_flash_latency);

    if (e_voltage_scaling::_2 == this->get_voltage_scaling() && e_voltage_scaling::_1 == new_voltage_scaling)
    {
        this->set_voltage_scaling(new_voltage_scaling);
        while (false == get_bit(PWR->SR2, PWR_SR2_VOSF_Pos));
    }

    if (new_flash_latency != this->get_flash_latency())
    {
        this->set_flash_latency(new_flash_latency);
        while (new_flash_latency != this->get_flash_latency());
    }

    set_flag(&(RCC->CFGR), static_cast<uint32>(a_source));
    clear_flag(&(RCC->CFGR), RCC_CFGR_HPRE);
    set_flag(&(RCC->CFGR), static_cast<uint32>(a_prescalers.ahb));

    while (false == is_flag(RCC->CFGR, static_cast<uint32>(a_source)));

    clear_flag(&(RCC->CFGR), RCC_CFGR_PPRE1);
    set_flag(&(RCC->CFGR), static_cast<uint32>(a_prescalers.apb1));

    clear_flag(&(RCC->CFGR), RCC_CFGR_PPRE2);
    set_flag(&(RCC->CFGR), static_cast<uint32>(a_prescalers.apb2));

    SystemCoreClock = a_frequency_hz;
}

void c_mcu::decrease_sysclk_frequency(e_sysclk_source a_source,
                                      uint32 a_frequency_hz,
                                      const s_bus_prescalers& a_prescalers)
{
    //RM0394
    //DocID027295 Rev 3
    //130, 5.1.7 Dynamic voltage scaling management
    //75, 3.3.3 Read access latency

    set_flag(&(RCC->CFGR), static_cast<uint32>(a_source));
    clear_flag(&(RCC->CFGR), RCC_CFGR_HPRE);
    set_flag(&(RCC->CFGR), static_cast<uint32>(a_prescalers.ahb));

    while (false == is_flag(RCC->CFGR, static_cast<uint32>(a_source)));

    auto new_voltage_scaling = this->select_voltage_scaling(a_frequency_hz);
    auto new_flash_latency   = this->select_flash_latency(a_frequency_hz, new_voltage_scaling);

    _assert(e_voltage_scaling::unkown != new_voltage_scaling);
    _assert(e_flash_latency::unknown != new_flash_latency);

    if (new_flash_latency != this->get_flash_latency())
    {
        this->set_flash_latency(new_flash_latency);
        while (new_flash_latency != this->get_flash_latency());
    }

    if (e_voltage_scaling::_1 == this->get_voltage_scaling() && e_voltage_scaling::_2 == new_voltage_scaling)
    {
        this->set_voltage_scaling(new_voltage_scaling);
    }

    clear_flag(&(RCC->CFGR), RCC_CFGR_PPRE1);
    set_flag(&(RCC->CFGR), static_cast<uint32>(a_prescalers.apb1));

    clear_flag(&(RCC->CFGR), RCC_CFGR_PPRE2);
    set_flag(&(RCC->CFGR), static_cast<uint32>(a_prescalers.apb2));

    SystemCoreClock = a_frequency_hz;
}

uint32 c_mcu::calculate_frequency_from_pll_configuration()
{
    uint32 pllm = (static_cast<uint32>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLM)) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    uint32 plln = (static_cast<uint32>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLN)) >> RCC_PLLCFGR_PLLN_Pos);

    uint32 pllvco = 0;

    switch (static_cast<e_pll_clock_source>(get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC)))
    {
        case e_pll_clock_source::msi:
        {
            _assert(true == this->is_clock_enabled(e_clock::msi));

            uint32 msi_range = get_flag(RCC->CR, RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
            pllvco = (msi_frequency_lut[msi_range] / pllm) * plln;
        }
        break;

        case e_pll_clock_source::hsi:
        {
            _assert(true == this->is_clock_enabled(e_clock::hsi));

            pllvco = (s_config::s_clock::hsi_frequency_hz / pllm) * plln;
        }
        break;
    }

    uint32 pllr = ((get_flag(RCC->PLLCFGR, RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1u) * 2u;
    return pllvco / pllr;
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml

#endif // STM32L452xx