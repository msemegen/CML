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
    set_flag(&(RCC->ICSCR), RCC_ICSCR_MSIRANGE, static_cast<uint32>(a_freq) << RCC_ICSCR_MSIRANGE_Pos);
    set_flag(&(RCC->CR), RCC_CR_MSION);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));

    clear_flag(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);

    while (false == is_flag(RCC->CR, RCC_CR_MSIRDY));
}

void c_mcu::enable_hsi_clock(e_hsi_frequency a_freq)
{
    _unused(a_freq);

    set_flag(&(RCC->CR), RCC_CR_HSION);

    while (false == is_flag(RCC->CR, RCC_CR_HSIRDY));
}

void c_mcu::enable_lsi_clock(e_lsi_frequency a_freq)
{
    _unused(a_freq);

    set_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (false == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
}

void c_mcu::disable_msi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_MSION);

    while (true == is_flag(RCC->CR, RCC_CR_MSIRDY));
}

void c_mcu::disable_hsi_clock()
{
    clear_flag(&(RCC->CR), RCC_CR_HSION);

    while (true == is_flag(RCC->CR, RCC_CR_HSIRDY));
}

void c_mcu::disable_lsi_clock()
{
    clear_flag(&(RCC->CSR), RCC_CSR_LSION);

    while (true == is_flag(RCC->CSR, RCC_CSR_LSIRDY));
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

    set_flag(&(RCC->CFGR), RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV,
                           static_cast<uint32>(a_source)                 |
                           static_cast<uint32>(a_pll_config.pll_divider) |
                           static_cast<uint32>(a_pll_config.pll_multiplier));

    set_flag(&(RCC->CR), RCC_CR_PLLON);

    while (false == is_flag(RCC->CR, RCC_CR_PLLRDY));
}

void c_mcu::disable_pll()
{
    clear_flag(&(RCC->CR), RCC_CR_PLLON);
    while (true == is_flag(RCC->CR, RCC_CR_PLLRDY));
}

void c_mcu::set_sysclk(e_sysclk_source a_source, const s_bus_prescalers& a_prescalers)
{
    if (nullptr != this->pre_sysclock_frequency_change_callback.p_function)
    {
        this->pre_sysclock_frequency_change_callback.p_function(this->pre_sysclock_frequency_change_callback.a_p_user_data);
    }

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
                this->increase_sysclk_frequency(e_sysclk_source::hsi,
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
                this->decrease_sysclk_frequency(e_sysclk_source::pll, pll_frequency_hz, a_prescalers);
            }
            else if (SystemCoreClock < pll_frequency_hz)
            {
                this->increase_sysclk_frequency(e_sysclk_source::pll, pll_frequency_hz, a_prescalers);
            }
        }
        break;
    }

    set_flag(&(FLASH->ACR), FLASH_ACR_PRFTEN | FLASH_ACR_PRE_READ);
    clear_flag(&(FLASH->ACR), FLASH_ACR_DISAB_BUF);

    if (nullptr != this->post_sysclock_frequency_change_callback.p_function)
    {
        this->post_sysclock_frequency_change_callback.p_function(this->post_sysclock_frequency_change_callback.a_p_user_data);
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

        case e_voltage_scaling::unknown:
        {
            _assert(false);
        }
        break;
    }

    return e_flash_latency::unknown;
}

c_mcu::e_voltage_scaling c_mcu::select_voltage_scaling(e_sysclk_source a_source, uint32 a_sysclk_freq)
{
    if (e_sysclk_source::msi == a_source || (e_sysclk_source::pll == a_source && a_sysclk_freq <= MHz(4)))
    {
        return e_voltage_scaling::_3;
    }

    if (e_sysclk_source::hsi == a_source || (e_sysclk_source::pll == a_source && a_sysclk_freq <= MHz(16)))
    {
        return e_voltage_scaling::_2;
    }

    if (e_sysclk_source::pll == a_source && a_sysclk_freq <= MHz(32))
    {
        return e_voltage_scaling::_3;
    }

    return e_voltage_scaling::unknown;
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

        case e_flash_latency::unknown:
        {
            _assert(false);
        }
        break;
    }
}

void c_mcu::set_voltage_scaling(e_voltage_scaling a_scaling)
{
    _assert(a_scaling != e_voltage_scaling::unknown);

    set_flag(&(PWR->CR), PWR_CR_VOS, static_cast<uint32>(a_scaling));
}

void c_mcu::set_sysclk_source(e_sysclk_source a_sysclk_source)
{
    set_flag(&(RCC->CFGR), RCC_CFGR_SW, static_cast<uint32>(a_sysclk_source));
    while (false == is_flag(RCC->CFGR, static_cast<uint32>(a_sysclk_source) << RCC_CFGR_SWS_Pos));
}

void c_mcu::set_bus_prescalers(const s_bus_prescalers& a_prescalers)
{
    _assert(s_bus_prescalers::e_ahb::unknown != a_prescalers.ahb);
    _assert(s_bus_prescalers::e_apb1::unknown != a_prescalers.apb1);
    _assert(s_bus_prescalers::e_apb2::unknown != a_prescalers.apb2);

    set_flag(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<uint32>(a_prescalers.ahb));
    set_flag(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<uint32>(a_prescalers.apb1));
    set_flag(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<uint32>(a_prescalers.apb2));
}

void c_mcu::increase_sysclk_frequency(e_sysclk_source a_source,
                                      uint32 a_frequency_hz,
                                      const s_bus_prescalers& a_prescalers)
{
    auto new_voltage_scaling = this->select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = this->select_flash_latency(a_frequency_hz, new_voltage_scaling);

    _assert(e_voltage_scaling::unknown != new_voltage_scaling);
    _assert(e_flash_latency::unknown != new_flash_latency);

    auto current_voltage_scaling = this->get_voltage_scaling();
    auto current_flash_latency   = this->get_flash_latency();

    if ((e_voltage_scaling::_3 == current_voltage_scaling && e_voltage_scaling::_2 == new_voltage_scaling) ||
        (e_voltage_scaling::_2 == current_voltage_scaling && e_voltage_scaling::_1 == new_voltage_scaling))
    {
        this->set_voltage_scaling(new_voltage_scaling);
        while (true == get_bit(PWR->CSR, PWR_CSR_VOSF_Pos));
    }

    if (e_flash_latency::_1 == current_flash_latency && e_flash_latency::_0 == new_flash_latency)
    {
        this->set_flash_latency(new_flash_latency);
        while (true == get_bit(FLASH->ACR, FLASH_ACR_LATENCY_Pos));
    }

    this->set_sysclk_source(a_source);
    this->set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

void c_mcu::decrease_sysclk_frequency(e_sysclk_source a_source,
                                      uint32 a_frequency_hz,
                                      const s_bus_prescalers& a_prescalers)
{
    this->set_sysclk_source(a_source);

    auto new_voltage_scaling = this->select_voltage_scaling(a_source, a_frequency_hz);
    auto new_flash_latency   = this->select_flash_latency(a_frequency_hz, new_voltage_scaling);

    _assert(e_voltage_scaling::unknown != new_voltage_scaling);
    _assert(e_flash_latency::unknown != new_flash_latency);

    auto current_voltage_scaling = this->get_voltage_scaling();
    auto current_flash_latency   = this->get_flash_latency();

    if ((e_voltage_scaling::_2 == current_voltage_scaling && e_voltage_scaling::_3 == new_voltage_scaling) ||
        (e_voltage_scaling::_1 == current_voltage_scaling && e_voltage_scaling::_2 == new_voltage_scaling))
    {
        this->set_voltage_scaling(new_voltage_scaling);
        while (true == get_bit(PWR->CSR, PWR_CSR_VOSF_Pos));
    }

    if (e_flash_latency::_0 == current_flash_latency && e_flash_latency::_1 == new_flash_latency)
    {
        this->set_flash_latency(new_flash_latency);
        while (false == get_bit(FLASH->ACR, FLASH_ACR_LATENCY_Pos));
    }

    this->set_bus_prescalers(a_prescalers);

    SystemCoreClock = a_frequency_hz;
}

uint32 c_mcu::calculate_frequency_from_pll_configuration()
{
    constexpr uint32 m_lut[] = { 3, 4, 6, 8, 12, 16, 24, 32, 48 };
    constexpr uint32 d_lut[] = { 2, 3, 4 };

    const uint32 mi = (get_flag(RCC->CFGR, RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_Pos) - 0;
    const uint32 di = (get_flag(RCC->CFGR, RCC_CFGR_PLLDIV) >> RCC_CFGR_PLLDIV_Pos) - 1;

    _assert(mi < sizeof(m_lut) / sizeof(m_lut[0]));
    _assert(di < sizeof(d_lut) / sizeof(d_lut[0]));

    return s_config::s_clock::hsi_frequency_hz / (true == is_flag(RCC->CR, RCC_CR_HSIDIVEN) ? 4 : 1) * m_lut[mi] / d_lut[di];
}

} // namespace stm32l011xx
} // namespace hal
} // namespace cml

#endif // STM32L011xx