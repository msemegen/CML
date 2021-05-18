#pragma once

/*
 *   Name: mcu.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l452xx.h>

// cml
#include <cml/bit_flag.hpp>

// soc
#include <soc/stm32l4/internal_flash.hpp>

namespace soc {
namespace stm32l4 {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

class mcu
{
public:
    enum class Clock : uint32_t
    {
        msi = RCC_CR_MSION,
        hsi = RCC_CR_HSION,
        pll = RCC_CR_PLLON,
        hsi48,
        lsi
    };

    enum class Sysclk_source : uint32_t
    {
        msi = RCC_CFGR_SW_MSI,
        hsi = RCC_CFGR_SW_HSI,
        pll = RCC_CFGR_SW_PLL,
    };

    enum class Clk48_mux_source : uint32_t
    {
        hsi48      = 0x0u,
        pll_sai1_q = RCC_CCIPR_CLK48SEL_0,
        pll_q      = RCC_CCIPR_CLK48SEL_1,
        msi        = RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1,
    };

    enum class Msi_frequency : uint32_t
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

    enum class Hsi_frequency : uint32_t
    {
        _16_MHz,
    };

    enum class Lsi_frequency : uint32_t
    {
        _32_kHz,
    };

    enum class Hsi48_frequency : uint32_t
    {
        _48_MHz
    };

    enum class Voltage_scaling : uint32_t
    {
        _1 = PWR_CR1_VOS_0,
        _2 = PWR_CR1_VOS_1,
    };

    enum class Reset_source : uint32_t
    {
        illegal_low_power    = RCC_CSR_LPWRRSTF,
        window_watchdog      = RCC_CSR_WWDGRSTF,
        independent_watchdog = RCC_CSR_IWDGRSTF,
        software             = RCC_CSR_SFTRSTF,
        bor                  = RCC_CSR_BORRSTF,
        pin                  = RCC_CSR_PINRSTF,
        option_byte_loader   = RCC_CSR_OBLRSTF,
        firewall             = RCC_CSR_FWRSTF
    };

    enum class FPU_mode : uint32_t
    {
        disabled               = 0x0u,
        privileged_access_only = 0xAu << 20u,
        enabled                = 0xFu << 20u,
    };

    struct Pll_config
    {
        enum class Source : uint32_t
        {
            msi = RCC_PLLCFGR_PLLSRC_MSI,
            hsi = RCC_PLLCFGR_PLLSRC_HSI,
            unknown
        };

        enum class M : uint32_t
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

        enum class Output : uint32_t
        {
            disabled = 0x0u,
            enabled  = 0x1u,
            unknown
        };

        struct PLL
        {
            struct R
            {
                enum class Divider : uint32_t
                {
                    _2 = 0,
                    _4 = RCC_PLLCFGR_PLLR_0,
                    _6 = RCC_PLLCFGR_PLLR_1,
                    _8 = RCC_PLLCFGR_PLLR_0 | RCC_PLLCFGR_PLLR_1,
                    unknown
                };

                Divider divider = Divider::unknown;
                Output output   = Output::unknown;
            };

            struct Q
            {
                enum class Divider : uint32_t
                {
                    _2 = 0,
                    _4 = RCC_PLLCFGR_PLLQ_0 >> RCC_PLLCFGR_PLLQ_Pos,
                    _6 = RCC_PLLCFGR_PLLQ_1 >> RCC_PLLCFGR_PLLQ_Pos,
                    _8 = (RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1) >> RCC_PLLCFGR_PLLQ_Pos,
                    unknown
                };

                Divider divider = Divider::unknown;
                Output output   = Output::unknown;
            };

            struct P
            {
                enum class Divider : uint32_t
                {
                    _7  = 0u,
                    _17 = RCC_PLLCFGR_PLLP_Msk,
                    unknown
                };

                Divider divider = Divider::unknown;
                Output output   = Output::unknown;
            };

            uint32_t n = 0;

            R r;
            Q q;
            P p;
        };

        struct PLLSAI1
        {
            struct R
            {
                enum class Divider : uint32_t
                {
                    _2 = 0,
                    _4 = RCC_PLLSAI1CFGR_PLLSAI1R_0,
                    _6 = RCC_PLLSAI1CFGR_PLLSAI1R_1,
                    _8 = RCC_PLLSAI1CFGR_PLLSAI1R,
                    unknown
                };

                Divider divider = Divider::unknown;
                Output output   = Output::unknown;
            };

            struct Q
            {
                enum class Divider : uint32_t
                {
                    _2 = 0,
                    _4 = RCC_PLLSAI1CFGR_PLLSAI1Q_0,
                    _6 = RCC_PLLSAI1CFGR_PLLSAI1Q_1,
                    _8 = RCC_PLLSAI1CFGR_PLLSAI1Q_0 | RCC_PLLSAI1CFGR_PLLSAI1Q_1,
                    unknown
                };

                Divider divider = Divider::unknown;
                Output output   = Output::unknown;
            };

            struct P
            {
                enum class Divider : uint32_t
                {
                    _7  = 0u,
                    _17 = RCC_PLLSAI1CFGR_PLLSAI1P_Msk,
                    unknown
                };

                Divider divider = Divider::unknown;
                Output output   = Output::unknown;
            };

            uint32_t n = 0;

            R r;
            Q q;
            P p;
        };

        Source source = Source::unknown;
        M m           = M::uknown;

        PLL pll;
        PLLSAI1 pllsai1;
    };

    struct Device_id
    {
        const uint8_t serial_number[12] = { 0 };
        const uint32_t type             = 0;
    };

    struct Sysclk_frequency_change_callback
    {
        using Function = void (*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_prescalers
    {
        enum class AHB : uint32_t
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

        enum class APB1 : uint32_t
        {
            _1  = RCC_CFGR_PPRE1_DIV1,
            _2  = RCC_CFGR_PPRE1_DIV2,
            _4  = RCC_CFGR_PPRE1_DIV4,
            _8  = RCC_CFGR_PPRE1_DIV8,
            _16 = RCC_CFGR_PPRE1_DIV16,
            unknown
        };

        enum class APB2 : uint32_t
        {
            _1  = RCC_CFGR_PPRE2_DIV1,
            _2  = RCC_CFGR_PPRE2_DIV2,
            _4  = RCC_CFGR_PPRE2_DIV4,
            _8  = RCC_CFGR_PPRE2_DIV8,
            _16 = RCC_CFGR_PPRE2_DIV16,
            unknown
        };

        AHB ahb   = AHB::unknown;
        APB1 apb1 = APB1::unknown;
        APB2 apb2 = APB2::unknown;
    };

    struct NVIC_config
    {
        enum class Grouping : uint32_t
        {
            _0 = 0x7,
            _1 = 0x6,
            _2 = 0x5,
            _3 = 0x4,
            _4 = 0x3,
            unknown
        };

        Grouping grouping      = Grouping::unknown;
        uint32_t base_priority = 0;
    };

public:
    static void enable_msi_clock(Msi_frequency a_freq);
    static void enable_hsi_clock(Hsi_frequency a_freq);
    static void enable_lsi_clock(Lsi_frequency a_freq);
    static void enable_hsi48_clock(Hsi48_frequency a_freq);

    static void disable_msi_clock();
    static void disable_hsi_clock();
    static void disable_lsi_clock();
    static void disable_hsi48_clock();

    static void enable_pll(const Pll_config& a_config);
    static void disable_pll();

    static void set_clk48_clock_mux_source(Clk48_mux_source a_source);

    static void
    set_sysclk(Sysclk_source a_source, const Bus_prescalers& a_prescalers, Voltage_scaling a_voltage_scaling);
    static void set_nvic(const NVIC_config& a_config);

    static void set_fpu_mode(FPU_mode a_mode)
    {
        cml::bit_flag::set(&(SCB->CPACR), ((3u << 10u * 2u) | (3u << 11u * 2u)), static_cast<uint32_t>(a_mode));
    }

    static void enable_dwt()
    {
        cml::bit_flag::set(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
        cml::bit_flag::set(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
    }

    static void disable_dwt()
    {
        cml::bit_flag::clear(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
        cml::bit_flag::clear(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
    }

    static void enable_syscfg()
    {
        cml::bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_SYSCFGEN);
    }

    static void disable_syscfg()
    {
        cml::bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_SYSCFGEN);
    }

    static void reset();
    static void halt();

    static void register_pre_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback);
    static void register_post_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback);

    static FPU_mode get_fpu_mode()
    {
        return static_cast<FPU_mode>(SCB->CPACR);
    }

    static bool is_dwt_enabled()
    {
        return true == cml::bit_flag::is(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk) &&
               cml::bit_flag::is(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);
    }

    static bool is_syscfg_enabled()
    {
        return cml::bit_flag::is(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    }

    static Bus_prescalers get_bus_prescalers();
    static Pll_config get_pll_config();

    static Clk48_mux_source get_clk48_mux_source()
    {
        return static_cast<Clk48_mux_source>(cml::bit_flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL));
    }

    static uint32_t get_clk48_mux_freqency_hz();

    static Device_id get_device_id()
    {
        const uint8_t* p_id_location = reinterpret_cast<uint8_t*>(UID_BASE);

        return { { p_id_location[0],
                   p_id_location[1],
                   p_id_location[2],
                   p_id_location[3],
                   p_id_location[4],
                   p_id_location[5],
                   p_id_location[6],
                   p_id_location[7],
                   p_id_location[8],
                   p_id_location[9],
                   p_id_location[10],
                   p_id_location[11] },

                 DBGMCU->IDCODE };
    }

    static Sysclk_source get_sysclk_source()
    {
        return static_cast<Sysclk_source>(cml::bit_flag::get(RCC->CFGR, RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos);
    }

    static uint32_t get_sysclk_frequency_hz()
    {
        return SystemCoreClock;
    }

    static constexpr uint32_t get_hsi_frequency_hz()
    {
        return 16u * 1000000u;
    }

    static constexpr uint32_t get_lsi_frequency_hz()
    {
        return 32u * 1000;
    }

    static bool is_clock_enabled(Clock a_clock)
    {
        switch (a_clock)
        {
            case Clock::msi:
            case Clock::hsi:
            case Clock::pll: {
                return cml::bit_flag::is(RCC->CR, static_cast<uint32_t>(a_clock));
            }
            break;

            case Clock::lsi: {
                return cml::bit_flag::is(RCC->CSR, RCC_CSR_LSION);
            }
            break;

            case Clock::hsi48: {
                return cml::bit_flag::is(RCC->CRRCR, RCC_CRRCR_HSI48ON);
            }
            break;
        }

        return false;
    }

    static Voltage_scaling get_voltage_scaling()
    {
        return static_cast<Voltage_scaling>(cml::bit_flag::get(PWR->CR1, PWR_CR1_VOS));
    }

    static Reset_source get_reset_source()
    {
        uint32_t flag = cml::bit_flag::get(RCC->CSR, 0xFB000000u);

        if (flag == 0x0u)
        {
            flag = RCC_CSR_PINRSTF;
        }

        cml::bit_flag::set(&(RCC->CSR), RCC_CSR_RMVF);

        return static_cast<Reset_source>(flag);
    }

private:
    mcu()           = delete;
    mcu(const mcu&) = delete;
    mcu(mcu&&)      = delete;
    ~mcu()          = delete;

    mcu& operator=(const mcu&) = delete;
    mcu& operator=(mcu&&) = delete;

    static internal_flash::Latency select_flash_latency(uint32_t a_syclk_freq, Voltage_scaling a_voltage_scaling);
    static Voltage_scaling select_voltage_scaling(Sysclk_source a_source, uint32_t a_sysclk_freq);

    static void set_flash_latency(internal_flash::Latency a_latency);
    static void set_voltage_scaling(Voltage_scaling a_scaling);
    static void set_sysclk_source(Sysclk_source a_sysclk_source);
    static void set_bus_prescalers(const Bus_prescalers& a_prescalers);

    static void increase_sysclk_frequency(Sysclk_source a_source,
                                          uint32_t a_frequency_hz,
                                          const Bus_prescalers& a_prescalers,
                                          Voltage_scaling a_voltage_scaling);

    static void decrease_sysclk_frequency(Sysclk_source a_source,
                                          uint32_t a_frequency_hz,
                                          const Bus_prescalers& a_prescalers,
                                          Voltage_scaling a_voltage_scaling);

    static uint32_t calculate_pll_r_output_frequency();
    static uint32_t calculate_pll_q_output_frequency();
    static uint32_t calculate_pllsai1_q_output_frequency();
};

#endif

} // namespace stm32l4
} // namespace soc