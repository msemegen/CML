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

class MCU
{
public:

    enum class Clock : common::uint32
    {
        MSI = RCC_CR_MSION,
        HSI = RCC_CR_HSION,
        PLL = RCC_CR_PLLON,
        LSI
    };

    enum class SYSCLK_source : common::uint32
    {
        MSI = RCC_CFGR_SW_MSI,
        HSI = RCC_CFGR_SW_HSI,
        PLL = RCC_CFGR_SW_PLL
    };

    enum class PLL_clock_source : common::uint32
    {
        HSI = RCC_CFGR_PLLSRC_HSI
    };

    enum class MSI_frequency : common::uint32
    {
        _65536_Hz  = 0,
        _131072_Hz = 1,
        _262144_Hz = 2,
        _524288_Hz = 3,
        _1048_kHz  = 4,
        _2097_kHz  = 5,
        _4194_kHz  = 6
    };

    enum class HSI_frequency : common::uint32
    {
        _16_MHz
    };

    enum class LSI_frequency : common::uint32
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

    struct PLL_config
    {
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

        bool hsidiv_enabled   = false;
        Multiplier multiplier = Multiplier::unknown;
        Divider divider       = Divider::unknown;
    };

    struct Id
    {
        const common::uint8  serial_number[config::mcu::DEVICE_ID_LENGTH] = { 0 };
        const common::uint32 type = 0;
    };

    struct SYSCLK_frequency_change_callback
    {
        void(*p_function)(void* a_p_user_data) = nullptr;
        void* a_p_user_data = nullptr;
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

    void enable_MSI_clock(MSI_frequency a_freq);
    void enable_HSI_clock(HSI_frequency a_freq);
    void enable_LSI_clock(LSI_frequency a_freq);

    void disable_MSI_clock();
    void disable_HSI_clock();
    void disable_LSI_clock();

    void enable_PLL(PLL_clock_source a_source, const PLL_config& a_pll_config);
    void disable_PLL();

    void set_SYSCLK(SYSCLK_source a_source, const Bus_prescalers& a_prescalers);

    Id get_id()
    {
        static_assert(12 == config::mcu::DEVICE_ID_LENGTH);

        common::uint8* p_id_location = reinterpret_cast<common::uint8*>(UID_BASE);

        return { { p_id_location[0], p_id_location[1], p_id_location[2],  p_id_location[3],
                   p_id_location[4], p_id_location[5], p_id_location[6],  p_id_location[7],
                   p_id_location[8], p_id_location[9], p_id_location[10], p_id_location[11] },

                  DBGMCU->IDCODE
        };
    }

    SYSCLK_source get_SYSCLK_source() const
    {
        return static_cast<SYSCLK_source>(common::get_flag(RCC->CFGR, RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos);
    }

    common::uint32 get_SYSCLK_frequency_hz() const
    {
        return SystemCoreClock;
    }

    bool is_clock_enabled(Clock a_clock) const
    {
        switch (a_clock)
        {
            case Clock::MSI:
            case Clock::HSI:
            case Clock::PLL:
            {
                return common::is_flag(RCC->CR, static_cast<common::uint32>(a_clock));
            }
            break;

            case Clock::LSI:
            {
                return common::is_flag(RCC->CSR, RCC_CSR_LSION);
            }
            break;
        }

        return false;
    }

    Voltage_scaling get_voltage_scaling() const
    {
        return static_cast<Voltage_scaling>(common::get_flag(PWR->CR, PWR_CR_VOS));
    }

    Flash_latency get_flash_latency() const
    {
        return static_cast<Flash_latency>(common::get_flag(FLASH->ACR, FLASH_ACR_LATENCY));
    }

    void register_pre_SYSCLK_frequency_change_callback(const SYSCLK_frequency_change_callback& a_callback)
    {
        this->pre_SYSCLK_frequency_change_callback = a_callback;
    }

    void register_post_sysclk_frequency_change_callback(const SYSCLK_frequency_change_callback& a_callback)
    {
        this->post_SYSCLK_frequency_change_callback = a_callback;
    }

    static MCU& get_instance()
    {
        static MCU instance;
        return instance;
    }

private:

    MCU()           = default;
    MCU(const MCU&) = delete;
    MCU(MCU&&)      = delete;
    ~MCU()          = default;

    MCU& operator = (const MCU&) = delete;
    MCU& operator = (MCU&&)      = delete;

    Flash_latency select_flash_latency(common::uint32 a_syclk_freq, Voltage_scaling a_voltage_scaling);
    Voltage_scaling select_voltage_scaling(SYSCLK_source a_source, common::uint32 a_sysclk_freq);

    void set_flash_latency(Flash_latency a_latency);
    void set_voltage_scaling(Voltage_scaling a_scaling);
    void set_SYSCLK_source(SYSCLK_source a_sysclk_source);
    void set_bus_prescalers(const Bus_prescalers& a_prescalers);

    void increase_SYSCLK_frequency(SYSCLK_source a_source,
                                   common::uint32 a_frequency_hz,
                                   const Bus_prescalers& a_prescalers);

    void decrease_SYSCLK_frequency(SYSCLK_source a_source,
                                   common::uint32 a_frequency_hz,
                                   const Bus_prescalers& a_prescalers);

    common::uint32 calculate_frequency_from_PLL_configuration();

private:

    SYSCLK_frequency_change_callback pre_SYSCLK_frequency_change_callback;
    SYSCLK_frequency_change_callback post_SYSCLK_frequency_change_callback;
};

} // namespace stm32l011xx
} // hal
} // cml