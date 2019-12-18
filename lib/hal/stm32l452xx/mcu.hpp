#pragma once

/*
    Name: mcu.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l452xx.h>

//cml
#include <common/bit.hpp>
#include <hal/stm32l452xx/config.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

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
        msi = RCC_CFGR_SW_MSI,
        hsi = RCC_CFGR_SW_HSI,
        pll = RCC_CFGR_SW_PLL,
    };

    enum class e_pll_clock_source : common::uint32
    {
        msi = RCC_PLLCFGR_PLLSRC_MSI,
        hsi = RCC_PLLCFGR_PLLSRC_HSI
    };

    enum class e_msi_frequency : common::uint32
    {
        _100_kHz = 0,
        _200_kHz = 1,
        _400_kHz = 2,
        _800_kHz = 3,
        _1_MHz   = 4,
        _2_MHz   = 5,
        _4_MHz   = 6,
        _8_MHz   = 7,
        _16_MHz  = 8,
        _24_MHz  = 9,
        _32_MHz  = 10,
        _48_MHz  = 11,
    };

    enum class e_hsi_frequency : common::uint32
    {
        _16_MHz,
    };

    enum class e_lsi_frequency : common::uint32
    {
        _32_kHz,
    };

    enum class e_flash_latency : common::uint32
    {
        _0 = FLASH_ACR_LATENCY_0WS,
        _1 = FLASH_ACR_LATENCY_1WS,
        _2 = FLASH_ACR_LATENCY_2WS,
        _3 = FLASH_ACR_LATENCY_3WS,
        _4 = FLASH_ACR_LATENCY_4WS,
        uknown
    };

    enum class e_voltage_scaling : common::uint32
    {
        _1 = PWR_CR1_VOS_0,
        _2 = PWR_CR1_VOS_1,
        uknown
    };

    struct s_pll_config
    {
        enum class e_m_divider : common::uint32
        {
            _1 = 0,
            _2 = RCC_PLLCFGR_PLLM_0,
            _3 = RCC_PLLCFGR_PLLM_1,
            _4 = RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0,
            _5 = RCC_PLLCFGR_PLLM_2,
            _6 = RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_0,
            _7 = RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_1,
            _8 = RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0,
            uknown
        };

        enum class e_r_divider : common::uint32
        {
            _2 = 0,
            _4 = RCC_PLLCFGR_PLLR_0,
            _6 = RCC_PLLCFGR_PLLR_1,
            _8 = RCC_PLLCFGR_PLLR,
            uknown
        };

        e_m_divider m_divider    = e_m_divider::uknown;
        common::uint32 n_divider = 0;
        e_r_divider r_divider    = e_r_divider::uknown;
    };

    struct s_id
    {
        const common::uint8  serial_number[s_config::s_mcu::device_id_length] = { 0 };
        const common::uint32 type = 0;
    };

    struct s_sysclk_frequency_change_callback
    {
        void(*p_function)(void* a_p_user_data) = nullptr;
        void* a_p_user_data                    = nullptr;
    };

    struct s_bus_prescalers
    {
        enum class e_ahb : common::uint32
        {
            _1   = RCC_CFGR_HPRE_DIV1,
            _2   = RCC_CFGR_HPRE_DIV2,
            _4   = RCC_CFGR_HPRE_DIV4,
            _8   = RCC_CFGR_HPRE_DIV8,
            _16  = RCC_CFGR_HPRE_DIV16,
            _64  = RCC_CFGR_HPRE_DIV64,
            _128 = RCC_CFGR_HPRE_DIV128,
            _256 = RCC_CFGR_HPRE_DIV256,
            _512 = RCC_CFGR_HPRE_DIV512,
            unknown
        };

        enum class e_apb1 : common::uint32
        {
            _1  = RCC_CFGR_PPRE1_DIV1,
            _2  = RCC_CFGR_PPRE1_DIV2,
            _4  = RCC_CFGR_PPRE1_DIV4,
            _8  = RCC_CFGR_PPRE1_DIV8,
            _16 = RCC_CFGR_PPRE1_DIV16,
            unknown
        };

        enum class e_apb2 : common::uint32
        {
            _1  = RCC_CFGR_PPRE2_DIV1,
            _2  = RCC_CFGR_PPRE2_DIV2,
            _4  = RCC_CFGR_PPRE2_DIV4,
            _8  = RCC_CFGR_PPRE2_DIV8,
            _16 = RCC_CFGR_PPRE2_DIV16,
            unknown
        };

        e_ahb  ahb  = e_ahb::unknown;
        e_apb1 apb1 = e_apb1::unknown;
        e_apb2 apb2 = e_apb2::unknown;
    };

    struct s_nvic
    {
        common::uint32 priority_grouping = 0;
        common::uint32 base_priority     = 0;
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

    void set_sysclk(e_sysclk_source a_source, const s_bus_prescalers& a_prescalers, const s_nvic& a_nvic_settings);
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
        return common::is_flag(PWR->CR1, PWR_CR1_LPR);
    }

    e_voltage_scaling get_voltage_scaling() const
    {
        return static_cast<e_voltage_scaling>(get_flag(PWR->CR1, PWR_CR1_VOS));
    }

    e_flash_latency get_flash_latency() const
    {
        return static_cast<e_flash_latency>(get_flag(FLASH->ACR, FLASH_ACR_LATENCY));
    }

    static c_mcu& get_instance()
    {
        static c_mcu instance;
        return instance;
    }

private:

    c_mcu()
        : clock_source(e_sysclk_source::msi)
        , enabled_clocks(static_cast<common::uint32>(e_clock::msi))
    {}

    c_mcu(const c_mcu&) = delete;
    c_mcu(c_mcu&&)      = delete;
    ~c_mcu()            = default;

    c_mcu& operator = (const c_mcu&) = delete;
    c_mcu& operator = (c_mcu&&)      = delete;

    e_flash_latency select_flash_latency(common::uint32 a_syclk_freq,
                                         e_voltage_scaling a_voltage_scaling);

    e_voltage_scaling select_voltage_scaling(common::uint32 a_sysclk_freq);

    void set_flash_latency(e_flash_latency a_latency);
    void set_voltage_scaling(e_voltage_scaling a_scaling);

    void increase_sysclk_frequency(e_sysclk_source a_source,
                                   common::uint32 a_frequency_hz,
                                   const s_bus_prescalers& a_prescalers);

    void decrease_sysclk_frequency(e_sysclk_source a_source,
                                   common::uint32 a_frequency_hz,
                                   const s_bus_prescalers& a_prescalers);

    common::uint32 calculate_frequency_from_pll_configuration();

private:

    e_sysclk_source clock_source;
    common::uint8   enabled_clocks;

    s_sysclk_frequency_change_callback pre_sysclock_frequency_callback;
    s_sysclk_frequency_change_callback post_sysclock_frequency_callback;
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml