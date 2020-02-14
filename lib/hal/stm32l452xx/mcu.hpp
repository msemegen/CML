#pragma once

/*
    Name: MCU.hpp

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

class mcu
{
public:

    enum class Clock : common::uint32
    {
        msi = RCC_CR_MSION,
        hsi = RCC_CR_HSION,
        pll = RCC_CR_PLLON,
        lsi
    };

    enum class Sysclk_source : common::uint32
    {
        msi = RCC_CFGR_SW_MSI,
        hsi = RCC_CFGR_SW_HSI,
        pll = RCC_CFGR_SW_PLL,
    };

    enum class Pll_clock_source : common::uint32
    {
        msi = RCC_PLLCFGR_PLLSRC_MSI,
        hsi = RCC_PLLCFGR_PLLSRC_HSI
    };

    enum class Msi_frequency : common::uint32
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

    enum class Hsi_frequency : common::uint32
    {
        _16_MHz,
    };

    enum class Lsi_frequency : common::uint32
    {
        _32_kHz,
    };

    enum class Flash_latency : common::uint32
    {
        _0 = FLASH_ACR_LATENCY_0WS,
        _1 = FLASH_ACR_LATENCY_1WS,
        _2 = FLASH_ACR_LATENCY_2WS,
        _3 = FLASH_ACR_LATENCY_3WS,
        _4 = FLASH_ACR_LATENCY_4WS,
        unknown
    };

    enum class Voltage_scaling : common::uint32
    {
        _1 = PWR_CR1_VOS_0,
        _2 = PWR_CR1_VOS_1,
        unkown
    };

    struct Pll_config
    {
        enum class M_divider : common::uint32
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

        enum class R_divider : common::uint32
        {
            _2 = 0,
            _4 = RCC_PLLCFGR_PLLR_0,
            _6 = RCC_PLLCFGR_PLLR_1,
            _8 = RCC_PLLCFGR_PLLR,
            uknown
        };

        M_divider m_divider      = M_divider::uknown;
        common::uint32 n_divider = 0;
        R_divider r_divider      = R_divider::uknown;
    };

    struct Id
    {
        const common::uint8  serial_number[config::mcu::device_id_length] = { 0 };
        const common::uint32 type = 0;
    };

    struct Sysclk_frequency_change_callback
    {
        void(*p_function)(void* a_p_user_data) = nullptr;
        void* a_p_user_data                    = nullptr;
    };

    struct Bus_prescalers
    {
        enum class AHB : common::uint32
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

        enum class APB1 : common::uint32
        {
            _1  = RCC_CFGR_PPRE1_DIV1,
            _2  = RCC_CFGR_PPRE1_DIV2,
            _4  = RCC_CFGR_PPRE1_DIV4,
            _8  = RCC_CFGR_PPRE1_DIV8,
            _16 = RCC_CFGR_PPRE1_DIV16,
            unknown
        };

        enum class APB2 : common::uint32
        {
            _1  = RCC_CFGR_PPRE2_DIV1,
            _2  = RCC_CFGR_PPRE2_DIV2,
            _4  = RCC_CFGR_PPRE2_DIV4,
            _8  = RCC_CFGR_PPRE2_DIV8,
            _16 = RCC_CFGR_PPRE2_DIV16,
            unknown
        };

        AHB  ahb  = AHB::unknown;
        APB1 apb1 = APB1::unknown;
        APB2 apb2 = APB2::unknown;
    };

    struct NVIC_config
    {
        common::uint32 priority_grouping = 0;
        common::uint32 base_priority     = 0;
    };

public:

    static void enable_msi_clock(Msi_frequency a_freq);
    static void enable_hsi_clock(Hsi_frequency a_freq);
    static void enable_lsi_clock(Lsi_frequency a_freq);

    static void disable_msi_clock();
    static void disable_hsi_clock();
    static void disable_lsi_clock();

    static void enable_pll(Pll_clock_source a_source, const Pll_config& a_pll_config);
    static void disable_pll();

    static void set_sysclk(Sysclk_source a_source, const Bus_prescalers& a_prescalers, const NVIC_config& a_NVIC_config);

    static void enable_dwt()
    {
        common::set_flag(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
        common::set_flag(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
    }

    static void disable_dwt()
    {
        common::clear_flag(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
        common::clear_flag(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
    }

    static void reset();
    static void halt();

    static void register_pre_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback);
    static void register_post_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback);

    static bool is_dwt_enabled()
    {
        return true == common::is_flag(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk) &&
                       common::is_flag(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);
    }

    static Id get_id()
    {
        static_assert(12 == config::mcu::device_id_length);

        common::uint8* p_id_location = reinterpret_cast<common::uint8*>(UID_BASE);

        return { { p_id_location[0], p_id_location[1], p_id_location[2],  p_id_location[3],
                   p_id_location[4], p_id_location[5], p_id_location[6],  p_id_location[7],
                   p_id_location[8], p_id_location[9], p_id_location[10], p_id_location[11] },

                  DBGMCU->IDCODE
        };
    }

    static Sysclk_source get_sysclk_source()
    {
        return static_cast<Sysclk_source>(common::get_flag(RCC->CFGR, RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos);
    }

    static common::uint32 get_syclk_frequency_hz()
    {
        return SystemCoreClock;
    }

    static bool is_clock_enabled(Clock a_clock)
    {
        switch (a_clock)
        {
            case Clock::msi:
            case Clock::hsi:
            case Clock::pll:
            {
                return common::is_flag(RCC->CR, static_cast<common::uint32>(a_clock));
            }
            break;

            case Clock::lsi:
            {
                return common::is_flag(RCC->CSR, RCC_CSR_LSION);
            }
            break;
        }

        return false;
    }

    static Voltage_scaling get_voltage_scaling()
    {
        return static_cast<Voltage_scaling>(common::get_flag(PWR->CR1, PWR_CR1_VOS));
    }

    static Flash_latency get_flash_latency()
    {
        return static_cast<Flash_latency>(common::get_flag(FLASH->ACR, FLASH_ACR_LATENCY));
    }

private:

    mcu()           = delete;
    mcu(const mcu&) = delete;
    mcu(mcu&&)      = delete;
    ~mcu()          = default;

    mcu& operator = (const mcu&) = delete;
    mcu& operator = (mcu&&)      = delete;

    static Flash_latency select_flash_latency(common::uint32 a_syclk_freq, Voltage_scaling a_voltage_scaling);
    static Voltage_scaling select_voltage_scaling(Sysclk_source a_source, common::uint32 a_sysclk_freq);

    static void set_flash_latency(Flash_latency a_latency);
    static void set_voltage_scaling(Voltage_scaling a_scaling);
    static void set_sysclk_source(Sysclk_source a_sysclk_source);
    static void set_bus_prescalers(const Bus_prescalers& a_prescalers);

    static void increase_sysclk_frequency(Sysclk_source a_source,
                                          common::uint32 a_frequency_hz,
                                          const Bus_prescalers& a_prescalers);

    static void decrease_sysclk_frequency(Sysclk_source a_source,
                                          common::uint32 a_frequency_hz,
                                          const Bus_prescalers& a_prescalers);

    static common::uint32 calculate_frequency_from_pll_configuration();
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml