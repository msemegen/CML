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
#include <common/frequency.hpp>
#include <common/integer.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

struct mcu
{
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
        pll = RCC_CFGR_SW_PLL
    };

    enum class Msi_frequency : common::uint32
    {
        _65536_Hz  = 0,
        _131072_Hz = 1,
        _262144_Hz = 2,
        _524288_Hz = 3,
        _1048_kHz  = 4,
        _2097_kHz  = 5,
        _4194_kHz  = 6
    };

    enum class Hsi_frequency : common::uint32
    {
        _16_MHz
    };

    enum class Lsi_frequency : common::uint32
    {
        _37_kHz
    };

    enum class Flash_latency : common::uint32
    {
        _0 = 0,
        _1 = FLASH_ACR_LATENCY,
        unknown
    };

    enum class Voltage_scaling : common::uint32
    {
        _1 = PWR_CR_VOS_0,
        _2 = PWR_CR_VOS_1,
        _3 = PWR_CR_VOS_0 | PWR_CR_VOS_1,
        unknown
    };

    enum class Reset_source : common::uint32
    {
        illegal_low_power           = RCC_CSR_LPWRRSTF,
        window_watchdog             = RCC_CSR_WWDGRSTF,
        independent_window_watchdog = RCC_CSR_IWDGRSTF,
        software                    = RCC_CSR_SFTRSTF,
        por_bdr                     = RCC_CSR_PORRSTF,
        pin                         = RCC_CSR_PINRSTF,
        option_byte_loader          = RCC_CSR_OBLRSTF
    };

    struct Pll_config
    {
        enum class Source : common::uint32
        {
            hsi = RCC_CFGR_PLLSRC_HSI
        };

        enum class Multiplier
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

        enum class Divider
        {
            _2 = RCC_CFGR_PLLDIV2,
            _3 = RCC_CFGR_PLLDIV3,
            _4 = RCC_CFGR_PLLDIV4,
            unknown
        };

        Source source;

        bool hsidiv_enabled   = false;
        Multiplier multiplier = Multiplier::unknown;
        Divider divider       = Divider::unknown;
    };

    struct Device_id
    {
        const common::uint8  serial_number[12] = { 0 };
        const common::uint32 type              = 0;
    };

    struct Sysclk_frequency_change_callback
    {
        using Function = void(*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_prescalers
    {
        enum class AHB
        {
            _1,
            _2,
            _4,
            _8,
            _16,
            _64,
            _128,
            _256,
            unknown
        };

        enum class APB1
        {
            _1,
            _2,
            _4,
            _8,
            _16,
            unknown
        };

        enum class APB2
        {
            _1,
            _2,
            _4,
            _8,
            _16,
            unknown
        };

        AHB  ahb  = AHB::unknown;
        APB1 apb1 = APB1::unknown;
        APB2 apb2 = APB2::unknown;
    };

public:

    static void enable_msi_clock(Msi_frequency a_freq);
    static void enable_hsi_clock(Hsi_frequency a_freq);
    static void enable_lsi_clock(Lsi_frequency a_freq);

    static void disable_msi_clock();
    static void disable_hsi_clock();
    static void disable_lsi_clock();

    static void enable_pll(const Pll_config& a_pll_config);
    static void disable_pll();

    static void set_sysclk(Sysclk_source a_source, const Bus_prescalers& a_prescalers);

    static void reset();
    static void halt();

    static void register_pre_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback);
    static void register_post_sysclk_frequency_change_callback(const Sysclk_frequency_change_callback& a_callback);

    static Bus_prescalers get_bus_prescalers();
    static Pll_config get_pll_config();

    static Device_id get_device_id()
    {
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

    static common::uint32 get_sysclk_frequency_hz()
    {
        return SystemCoreClock;
    }

    static constexpr common::uint32 get_hsi_frequency_hz()
    {
        return common::MHz(16u);
    }

    static constexpr common::uint32 get_lsi_frequency_hz()
    {
        return common::kHz(37u);
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
        return static_cast<Voltage_scaling>(common::get_flag(PWR->CR, PWR_CR_VOS));
    }

    static Flash_latency get_flash_latency()
    {
        return static_cast<Flash_latency>(common::get_flag(FLASH->ACR, FLASH_ACR_LATENCY));
    }

    static Reset_source get_reset_source()
    {
        uint32_t flag = common::get_flag(RCC->CSR, 0xFB000000u);

        if (flag == 0x0u)
        {
            flag = RCC_CSR_PINRSTF;
        }

        common::set_flag(&(RCC->CSR), RCC_CSR_RMVF);

        return static_cast<Reset_source>(flag);
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

} // namespace stm32l011xx
} // hal
} // cml