#pragma once

/*
 *   Name: mcu.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {

class mcu : private cml::Non_constructible
{
public:
    enum class FPU_mode : uint32_t
    {
        disabled               = 0x0u,
        privileged_access_only = 0xAu << 20u,
        enabled                = 0xFu << 20u,
    };

    enum class DWT_mode : uint32_t
    {
        disabled,
        enabled
    };

    struct Id
    {
        const uint8_t serial_number[12] = { 0 };
        const uint32_t type             = 0;
    };

public:
    static void halt()
    {
        __disable_irq();
        while (true)
            ;
    }

    static Id get_id()
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

    static void set_DWT_mode(DWT_mode a_mode)
    {
        switch (a_mode)
        {
            case DWT_mode::enabled: {
                cml::bit_flag::set(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
                cml::bit_flag::set(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
            }
            break;

            case DWT_mode::disabled: {
                cml::bit_flag::clear(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
                cml::bit_flag::clear(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
            }
            break;
        }
    }

    static void set_FPU_mode(FPU_mode a_mode)
    {
        cml::bit_flag::set(&(SCB->CPACR), ((3u << 10u * 2u) | (3u << 11u * 2u)), static_cast<uint32_t>(a_mode));
    }

    static DWT_mode get_DWT_mode()
    {
        return static_cast<DWT_mode>(cml::bit_flag::is(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk) &&
                                     cml::bit_flag::is(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk));
    }

    static FPU_mode get_FPU_mode()
    {
        return static_cast<FPU_mode>(SCB->CPACR);
    }
};

template<> class rcc<mcu> : private cml::Non_constructible
{
public:
    enum class Clock : uint32_t
    {
        MSI = RCC_CR_MSION,
        HSI = RCC_CR_HSION,
        PLL = RCC_CR_PLLON,
        HSI48,
        LSI
    };

    enum class SYSCLK_source : uint32_t
    {
        MSI = RCC_CFGR_SW_MSI,
        HSI = RCC_CFGR_SW_HSI,
        PLL = RCC_CFGR_SW_PLL,
    };

    enum class SYSCFG_mode : uint32_t
    {
        disabled,
        enabled
    };

    enum class CLK48_source : uint32_t
    {
        HSI48 = 0x0u,
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        PLL_SAI1_Q = RCC_CCIPR_CLK48SEL_0,
#endif
        PLL_Q = RCC_CCIPR_CLK48SEL_1,
        MSI   = RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1,
    };

    enum class MSI_frequency : uint32_t
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

    enum class HSI_frequency : uint32_t
    {
        _16_MHz,
    };

    enum class LSI_frequency : uint32_t
    {
        _32_kHz,
    };

    enum class HSI48_frequency : uint32_t
    {
        _48_MHz
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

    enum class PLL_source : uint32_t
    {
        MSI = RCC_PLLCFGR_PLLSRC_MSI,
        HSI = RCC_PLLCFGR_PLLSRC_HSI,
    };

    enum class PLLM
    {
        _1 = 0,
        _2 = RCC_PLLCFGR_PLLM_0,
        _3 = RCC_PLLCFGR_PLLM_1,
        _4 = RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0,
        _5 = RCC_PLLCFGR_PLLM_2,
        _6 = RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_0,
        _7 = RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_1,
        _8 = RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0,
    };

    struct MCO_config
    {
        enum class Source : uint32_t
        {
            SYSCLK = 0x1000000u,
            MSI    = 0x2000000u,
            HSI    = 0x3000000u,
            PLL    = 0x5000000u,
            LSI    = 0x6000000u
        };

        enum class Divider
        {
            _1  = 0x00000000u,
            _2  = 0x10000000u,
            _4  = 0x20000000u,
            _8  = 0x30000000u,
            _16 = 0x40000000u
        };

        Source source   = cml::various::get_enum_incorrect_value<Source>();
        Divider divider = cml::various::get_enum_incorrect_value<Divider>();
    };

    struct PLL_config
    {
        enum class Output : uint32_t
        {
            disabled = 0x0u,
            enabled  = 0x1u,
        };

        struct R
        {
            enum class Divider : uint32_t
            {
                _2 = 0,
                _4 = RCC_PLLCFGR_PLLR_0,
                _6 = RCC_PLLCFGR_PLLR_1,
                _8 = RCC_PLLCFGR_PLLR_0 | RCC_PLLCFGR_PLLR_1,
            };

            Divider divider = cml::various::get_enum_incorrect_value<Divider>();
            Output output   = cml::various::get_enum_incorrect_value<Output>();
        };

        struct Q
        {
            enum class Divider : uint32_t
            {
                _2 = 0,
                _4 = RCC_PLLCFGR_PLLQ_0 >> RCC_PLLCFGR_PLLQ_Pos,
                _6 = RCC_PLLCFGR_PLLQ_1 >> RCC_PLLCFGR_PLLQ_Pos,
                _8 = (RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1) >> RCC_PLLCFGR_PLLQ_Pos,
            };

            Divider divider = cml::various::get_enum_incorrect_value<Divider>();
            Output output   = cml::various::get_enum_incorrect_value<Output>();
        };
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        struct P
        {
            enum class Divider : uint32_t
            {
                _7  = 0u,
                _17 = RCC_PLLCFGR_PLLP_Msk,
            };

            Divider divider = cml::various::get_enum_incorrect_value<Divider>();
            Output output   = cml::various::get_enum_incorrect_value<Output>();
        };
#endif
        uint32_t n = 0;

        R r;
        Q q;
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        P p;
#endif
    };

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    struct PLLSAI1_config
    {
        enum class Output : uint32_t
        {
            disabled = 0x0u,
            enabled  = 0x1u,
        };

        struct R
        {
            enum class Divider : uint32_t
            {
                _2 = 0,
                _4 = RCC_PLLSAI1CFGR_PLLSAI1R_0,
                _6 = RCC_PLLSAI1CFGR_PLLSAI1R_1,
                _8 = RCC_PLLSAI1CFGR_PLLSAI1R,
            };

            Divider divider = cml::various::get_enum_incorrect_value<Divider>();
            Output output   = cml::various::get_enum_incorrect_value<Output>();
        };

        struct Q
        {
            enum class Divider : uint32_t
            {
                _2 = 0,
                _4 = RCC_PLLSAI1CFGR_PLLSAI1Q_0,
                _6 = RCC_PLLSAI1CFGR_PLLSAI1Q_1,
                _8 = RCC_PLLSAI1CFGR_PLLSAI1Q_0 | RCC_PLLSAI1CFGR_PLLSAI1Q_1,
            };

            Divider divider = cml::various::get_enum_incorrect_value<Divider>();
            Output output   = cml::various::get_enum_incorrect_value<Output>();
        };

        struct P
        {
            enum class Divider : uint32_t
            {
                _7  = 0u,
                _17 = RCC_PLLSAI1CFGR_PLLSAI1P_Msk,
            };

            Divider divider = cml::various::get_enum_incorrect_value<Divider>();
            Output output   = cml::various::get_enum_incorrect_value<Output>();
        };

        uint32_t n = 0;

        R r;
        Q q;
        P p;
    };
#endif

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
        };

        enum class APB1 : uint32_t
        {
            _1  = RCC_CFGR_PPRE1_DIV1,
            _2  = RCC_CFGR_PPRE1_DIV2,
            _4  = RCC_CFGR_PPRE1_DIV4,
            _8  = RCC_CFGR_PPRE1_DIV8,
            _16 = RCC_CFGR_PPRE1_DIV16,
        };

        enum class APB2 : uint32_t
        {
            _1  = RCC_CFGR_PPRE2_DIV1,
            _2  = RCC_CFGR_PPRE2_DIV2,
            _4  = RCC_CFGR_PPRE2_DIV4,
            _8  = RCC_CFGR_PPRE2_DIV8,
            _16 = RCC_CFGR_PPRE2_DIV16,
        };

        AHB ahb   = cml::various::get_enum_incorrect_value<AHB>();
        APB1 apb1 = cml::various::get_enum_incorrect_value<APB1>();
        APB2 apb2 = cml::various::get_enum_incorrect_value<APB2>();
    };

public:
    static void enable_clock(Clock a_clock, MSI_frequency a_freq);
    static void enable_clock(Clock a_clock, HSI_frequency a_freq);
    static void enable_clock(Clock a_clock, LSI_frequency a_freq);
    static void enable_clock(Clock a_clock, HSI48_frequency a_freq);
    static void enable_clock(Clock a_clock, PLL_source a_source, PLLM a_m, const PLL_config& a_config);
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    static void enable_clock(Clock a_clock,
                             PLL_source a_source,
                             PLLM a_m,
                             const PLL_config& a_pll_config,
                             const PLLSAI1_config& a_pllsai1_config);
#endif

    static void disable_clock(Clock a_clock);

    static void enable_MCO(const MCO_config& a_config);
    static void disable_MCO();
    static MCO_config get_MCO_config();

    static void set_CLK48_source(CLK48_source a_source);
    static void set_SYSCLK_source(SYSCLK_source a_source, const Bus_prescalers& a_prescalers);

    static void set_SYSCFG_mode(SYSCFG_mode a_mode)
    {
        switch (a_mode)
        {
            case SYSCFG_mode::enabled: {
                cml::bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_SYSCFGEN);
            }
            break;

            case SYSCFG_mode::disabled: {
                cml::bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_SYSCFGEN);
            }
            break;
        }
    }

    static SYSCFG_mode get_SYSCFG_mode()
    {
        return static_cast<SYSCFG_mode>(cml::bit_flag::is(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN));
    }

    static Bus_prescalers get_bus_prescalers();
    static PLL_config get_PLL_config();
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    static PLLSAI1_config get_PLLSAI1_config();
#endif

    static CLK48_source get_CLK48_source()
    {
        return static_cast<CLK48_source>(cml::bit_flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL));
    }

    static SYSCLK_source get_SYSCLK_source()
    {
        return static_cast<SYSCLK_source>(cml::bit_flag::get(RCC->CFGR, RCC_CFGR_SW) >> RCC_CFGR_SW_Pos);
    }

    static uint32_t get_clock_frequency_Hz(Clock a_clock);
    static uint32_t get_CLK48_frequency_Hz();
    static uint32_t get_SYSCLK_frequency_Hz();

    static bool is_clock_enabled(Clock a_clock);
    static Reset_source get_reset_source();

private:

    static uint32_t calculate_PLL_R_output_frequency();
    static uint32_t calculate_PLL_Q_output_frequency();
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    static uint32_t calculate_PLLSAI1_Q_output_frequency();
#endif
};

} // namespace stm32l4
} // namespace m4
} // namespace soc