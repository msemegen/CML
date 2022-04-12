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
#include <soc/m4/stm32l4/defs.hpp>
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
    enum class FPU_mode : std::uint32_t
    {
        disabled               = 0x000000u,
        privileged_access_only = 0xA00000u,
        enabled                = 0xF00000u,
    };
    enum class DWT_mode : std::uint32_t
    {
        disabled,
        enabled
    };
    enum class SYSCFG_mode : std::uint32_t
    {
        disabled,
        enabled
    };

    enum class Package : std::uint32_t
    {
        LQFP64   = 0x0u,
        WLCSP64  = 0x1u,
        LQFP100  = 0x2u,
        WLCSP36  = 0x5u,
        UFQFPN32 = 0x8u,
        LQFP32   = 0x9u,
        UFQFPN48 = 0xAu,
        LQFP48   = 0xBu,
        WLCSP49  = 0xCu,
        UFBGA64  = 0xDu,
        UFBGA100 = 0xEu,
#if defined(SOC_PACKAGE_WITH_EXTERNAL_SMPS_PRESENT)
        WLCSP36_with_external_SMPS = 0xFu,
        LQFP64_with_external_SMPS  = 0x16u
#endif
    };

    struct Id
    {
        std::uint8_t serial_number[12] = { 0 };
        std::uint32_t type             = 0;
    };

    static void halt()
    {
        __disable_irq();
        __builtin_trap();

        while (true)
            ;
    }

    static Id get_id()
    {
        const std::uint8_t* p_id_location = reinterpret_cast<std::uint8_t*>(UID_BASE);

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

    static void set_SYSCFG_mode(SYSCFG_mode a_mode)
    {
        cml::bit_flag::set(
            &(RCC->APB2ENR), RCC_APB2ENR_SYSCFGEN, SYSCFG_mode::enabled == a_mode ? RCC_APB2ENR_SYSCFGEN : 0x0u);
    }

    static DWT_mode get_DWT_mode()
    {
        return static_cast<DWT_mode>(cml::bit_flag::is(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk));
    }

    static FPU_mode get_FPU_mode()
    {
        return static_cast<FPU_mode>(SCB->CPACR);
    }

    static SYSCFG_mode get_SYSCFG_mode()
    {
        return static_cast<SYSCFG_mode>(cml::bit_flag::is(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN));
    }

    static constexpr Package get_package()
    {
        return static_cast<Package>(*(reinterpret_cast<std::uint32_t*>(PACKAGE_BASE)));
    }

    static bool is_in_debug_mode()
    {
        return cml::bit_flag::is(CoreDebug->DHCSR, CoreDebug_DHCSR_C_DEBUGEN_Msk);
    }
};
template<> class rcc<mcu> : private cml::Non_constructible
{
public:
    struct Bus_prescalers
    {
        enum class AHB : std::uint32_t
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

        enum class APB1 : std::uint32_t
        {
            _1  = RCC_CFGR_PPRE1_DIV1,
            _2  = RCC_CFGR_PPRE1_DIV2,
            _4  = RCC_CFGR_PPRE1_DIV4,
            _8  = RCC_CFGR_PPRE1_DIV8,
            _16 = RCC_CFGR_PPRE1_DIV16,
        };

        enum class APB2 : std::uint32_t
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

    class MSI : private cml::Non_constructible
    {
    public:
        enum class Frequency : std::uint32_t
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

        static void enable(Frequency a_frequency);
        static void disable();

        static bool is_enabled()
        {
            return cml::bit_flag::is(RCC->CR, RCC_CR_MSION);
        }

        static std::uint32_t get_frequency_Hz();
    };
    class HSI16 : private cml::Non_constructible
    {
    public:
        enum class Frequency : std::uint32_t
        {
            _16_MHz
        };

        static void enable(Frequency a_frequency);
        static void disable();

        static bool is_enabled()
        {
            return cml::bit_flag::is(RCC->CR, RCC_CR_HSION);
        }

        static std::uint32_t get_frequency_Hz()
        {
            if (true == is_enabled())
            {
                return 16_MHz;
            }

            return 0u;
        }
    };
    class HSI48 : private cml::Non_constructible
    {
    public:
        enum class Frequency : std::uint32_t
        {
            _48_MHz
        };

        static void enable(Frequency a_frequency);
        static void disable();

        static bool is_enabled()
        {
            return cml::bit_flag::is(RCC->CRRCR, RCC_CRRCR_HSI48ON);
        }

        static std::uint32_t get_frequency_Hz()
        {
            if (true == is_enabled())
            {
                return 48_MHz;
            }

            return 0u;
        }
    };
    class PLL : private cml::Non_constructible
    {
    public:
        enum class M : std::uint32_t
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

        struct RQP
        {
            enum class Output : std::uint32_t
            {
                disabled = 0x0u,
                enabled  = 0x1u,
            };

            struct R
            {
                enum class Divider : std::uint32_t
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
                enum class Divider : std::uint32_t
                {
                    _2 = 0,
                    _4 = RCC_PLLCFGR_PLLQ_0 >> RCC_PLLCFGR_PLLQ_Pos,
                    _6 = RCC_PLLCFGR_PLLQ_1 >> RCC_PLLCFGR_PLLQ_Pos,
                    _8 = (RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1) >> RCC_PLLCFGR_PLLQ_Pos,
                };

                Divider divider = cml::various::get_enum_incorrect_value<Divider>();
                Output output   = cml::various::get_enum_incorrect_value<Output>();
            };

            struct P
            {
                enum class Divider : std::uint32_t
                {
                    _7  = 0u,
                    _17 = RCC_PLLCFGR_PLLP_Msk,
                };

                Divider divider = cml::various::get_enum_incorrect_value<Divider>();
                Output output   = cml::various::get_enum_incorrect_value<Output>();
            };

            std::uint32_t n = 0;

            R r;
            Q q;
            P p;
        };
        struct RQPSAI1
        {
            enum class Output : std::uint32_t
            {
                disabled = 0x0u,
                enabled  = 0x1u,
            };

            struct R
            {
                enum class Divider : std::uint32_t
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
                enum class Divider : std::uint32_t
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
                enum class Divider : std::uint32_t
                {
                    _7  = 0u,
                    _17 = RCC_PLLSAI1CFGR_PLLSAI1P_Msk
                };

                Divider divider = cml::various::get_enum_incorrect_value<Divider>();
                Output output   = cml::various::get_enum_incorrect_value<Output>();
            };

            std::uint32_t n = 0;

            R r;
            Q q;
            P p;
        };

#if defined(SOC_PLLSAI_PRESENT)
        template<typename Source_t> static void enable(M a_M, const RQP a_RQP, const RQPSAI1& a_RQPSAI1) = delete;
#endif

#if !defined(SOC_PLLSAI_PRESENT)
        template<typename Source_t> static void enable(M a_M, const RQP a_RQP) = delete;
#endif
        static void disable();

        static bool is_enabled()
        {
            return cml::bit_flag::is(RCC->CR, RCC_CR_PLLON);
        }

        static std::uint32_t get_frequency_Hz();
    };
    class LSI : private cml::Non_constructible
    {
    public:
        enum class Frequency : std::uint32_t
        {
            _32_kHz,
        };

        static void enable(Frequency a_frequency);
        static void disable();

        static bool is_enabled()
        {
            return cml::bit_flag::is(RCC->CSR, RCC_CSR_LSION);
        }

        static std::uint32_t get_frequency_Hz()
        {
            if (true == is_enabled())
            {
                return 32_kHz;
            }

            return 0u;
        }
    };
    class CLK48_mux : private cml::Non_constructible
    {
    public:
        template<typename Source_t> static void set_source() = delete;
        static std::uint32_t get_frequency_Hz();
    };

    template<typename Source_t> static void set_SYSCLK_source(const Bus_prescalers& a_prescalers) = delete;

    static std::uint32_t get_SYSCLK_frequency_Hz();
    static std::uint32_t get_HCLK_frequency_Hz();
    static std::uint32_t get_PCLK1_frequency_Hz();
    static std::uint32_t get_PCLK2_frequency_Hz();
};

#if defined(SOC_PLLSAI_PRESENT)
template<> void rcc<mcu>::PLL::enable<rcc<mcu>::MSI>(M a_M, const RQP a_RQP, const RQPSAI1& a_RQPSAI1);
template<> void rcc<mcu>::PLL::enable<rcc<mcu>::HSI16>(M a_M, const RQP a_RQP, const RQPSAI1& a_RQPSAI1);
#endif

#if !defined(SOC_PLLSAI_PRESENT)
template<> void rcc<mcu>::PLL::enable<rcc<mcu>::MSI>(M a_M, const RQP a_RQP);
template<> void rcc<mcu>::PLL::enable<rcc<mcu>::HSI>(M a_M, const RQP a_RQP);
#endif

template<> void rcc<mcu>::CLK48_mux::set_source<rcc<mcu>::HSI48>();
template<> void rcc<mcu>::CLK48_mux::set_source<rcc<mcu>::MSI>();
template<> void rcc<mcu>::CLK48_mux::set_source<rcc<mcu>::PLL::RQP::Q>();
#if defined(SOC_PLLSAI_PRESENT)
template<> void rcc<mcu>::CLK48_mux::set_source<rcc<mcu>::PLL::RQPSAI1::Q>();
#endif

template<> void rcc<mcu>::set_SYSCLK_source<rcc<mcu>::MSI>(const Bus_prescalers& a_prescalers);
template<> void rcc<mcu>::set_SYSCLK_source<rcc<mcu>::HSI16>(const Bus_prescalers& a_prescalers);
template<> void rcc<mcu>::set_SYSCLK_source<rcc<mcu>::PLL>(const Bus_prescalers& a_prescalers);
} // namespace stm32l4
} // namespace m4
} // namespace soc