#pragma once

/*
    Name: mcu.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l0xx.h>

//cml
#include <common/bit.hpp>
#include <common/integer.hpp>
#include <hal/stm32l011xx/config.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

class c_mcu
{
public:

    enum class e_clock : common::uint32
    {
        msi = 1,
        hsi = 2,
        lsi = 3,
        pll = 4
    };

    enum class e_sysclk_source : common::uint32
    {
        msi = RCC_CFGR_SWS_MSI,
        hsi = RCC_CFGR_SWS_HSI,
        pll = RCC_CFGR_SWS_PLL
    };

    enum class e_pll_clock_source : common::uint32
    {
        hsi = RCC_CFGR_PLLSRC_HSI
    };

    enum class e_msi_frequency : common::uint32
    {
        _65536_Hz  = 0,
        _131072_Hz = 1,
        _262144_Hz = 2,
        _524288_Hz = 3,
        _1048_kHz  = 4,
        _2097_kHz  = 5,
        _4194_kHz  = 6
    };

    enum class e_hsi_frequency : common::uint32
    {
        _16_MHz
    };

    enum class e_lsi_frequency : common::uint32
    {
        _37_kHz
    };

    enum class e_flash_latency : common::uint32
    {
        _0 = 0,
        _1 = FLASH_ACR_LATENCY,
        uknown
    };

    enum class e_voltage_scaling : common::uint32
    {
        _1 = PWR_CR_VOS_0,
        _2 = PWR_CR_VOS_1,
        uknown
    };

    struct s_pll_config
    {
        enum class e_pll_multiplier
        {
            _3 = RCC_CFGR_PLLMUL3,
            _4 = RCC_CFGR_PLLMUL4,
            _6 = RCC_CFGR_PLLMUL6,
            _8 = RCC_CFGR_PLLMUL8,
            _12 = RCC_CFGR_PLLMUL12,
            _16 = RCC_CFGR_PLLMUL16,
            _24 = RCC_CFGR_PLLMUL24,
            _32 = RCC_CFGR_PLLMUL32,
            _48 = RCC_CFGR_PLLMUL48,
            unknown
        };

        enum class e_pll_divider
        {
            _2 = RCC_CFGR_PLLDIV2,
            _3 = RCC_CFGR_PLLDIV3,
            _4 = RCC_CFGR_PLLDIV4,
            unknown
        };

        bool hsi_divader                = false;
        e_pll_multiplier pll_multiplier = e_pll_multiplier::unknown;
        e_pll_divider pll_divider       = e_pll_divider::unknown;
    };

    struct s_id
    {
        const common::uint8  serial_number[s_config::s_mcu::device_id_length] = { 0 };
        const common::uint32 type = 0;
    };

    struct s_sysclk_frequency_change_callback
    {
        void(*p_function)(void* a_p_user_data) = nullptr;
        void* a_p_user_data = nullptr;
    };

    struct s_bus_prescalers
    {

    };

public:

    void enable_msi_clock(e_msi_frequency a_freq);
    void enable_hsi_clock(e_hsi_frequency a_freq);
    void enable_lsi_clock(e_lsi_frequency a_freq);

    void disable_msi_clock();
    void disable_hsi_clock();
    void disable_lsi_clock();

    void enable_pll(e_pll_clock_source a_source, const s_pll_config& a_pll_config);
    void disable_pll();

    void set_sysclk(e_sysclk_source a_source, const s_bus_prescalers& a_prescalers);
    bool enable_low_power_run();
    void disable_low_power_run();

    s_id get_id()
    {
        static_assert(12 == s_config::s_mcu::device_id_length);

        common::uint8* p_id_location = reinterpret_cast<common::uint8*>(UID_BASE);

        return { { p_id_location[0], p_id_location[1], p_id_location[2],  p_id_location[3],
                   p_id_location[4], p_id_location[5], p_id_location[6],  p_id_location[7],
                   p_id_location[8], p_id_location[9], p_id_location[10], p_id_location[11] },

                  DBGMCU->IDCODE
        };
    }

    e_sysclk_source get_sysclk_source() const
    {
        return this->clock_source;
    }

    common::uint32 get_syclk_frequency_hz() const
    {
        return SystemCoreClock;
    }

    bool is_clock_enabled(e_clock a_clock) const
    {
        return common::get_bit(this->enabled_clocks, static_cast<common::uint32>(a_clock));
    }

    bool is_in_low_power_run() const
    {
        return common::is_flag(PWR->CR, PWR_CR_LPRUN);
    }

    e_voltage_scaling get_voltage_scaling() const;
    e_flash_latency get_flash_latency() const;

    static c_mcu& get_instance()
    {
        static c_mcu instance;
        return instance;
    }

private:

    e_sysclk_source clock_source;
    common::uint8   enabled_clocks;

    s_sysclk_frequency_change_callback pre_sysclock_frequency_callback;
    s_sysclk_frequency_change_callback post_sysclock_frequency_callback;
};

} // namespace stm32l011xx
} // hal
} // cml