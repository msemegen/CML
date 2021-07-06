#pragma once

/*
 *   Name: mcu.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/bit_flag.hpp>
#include <cml/various.hpp>

// externals
#ifdef STM32L4
#include <stm32l4xx.h>
#endif

namespace soc {
namespace m4 {

#ifdef STM32L4
class mcu
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

    struct NVIC_config
    {
        enum class Grouping : uint32_t
        {
            _0 = 0x7,
            _1 = 0x6,
            _2 = 0x5,
            _3 = 0x4,
            _4 = 0x3,
        };

        Grouping grouping      = cml::various::get_enum_incorrect_value<Grouping>();
        uint32_t base_priority = 0;
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

    static void set_dwt_mode(DWT_mode a_mode)
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

    static void set_fpu_mode(FPU_mode a_mode)
    {
        cml::bit_flag::set(&(SCB->CPACR), ((3u << 10u * 2u) | (3u << 11u * 2u)), static_cast<uint32_t>(a_mode));
    }

    static void set_nvic_config(const NVIC_config& a_config)
    {
        NVIC_SetPriorityGrouping(static_cast<uint32_t>(a_config.grouping));
        __set_BASEPRI(a_config.base_priority);
    }

    static DWT_mode get_dwt_mode()
    {
        return static_cast<DWT_mode>(cml::bit_flag::is(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk) &&
                                     cml::bit_flag::is(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk));
    }

    static FPU_mode get_fpu_mode()
    {
        return static_cast<FPU_mode>(SCB->CPACR);
    }

    static NVIC_config get_nvic_config()
    {
        return { static_cast<NVIC_config::Grouping>(NVIC_GetPriorityGrouping()), __get_BASEPRI() };
    }

private:
    mcu()           = delete;
    mcu(const mcu&) = delete;
    mcu(mcu&&)      = delete;
    ~mcu()          = delete;

    mcu& operator=(const mcu&) = delete;
    mcu& operator=(mcu&&) = delete;
};

#endif

} // namespace m4
} // namespace soc