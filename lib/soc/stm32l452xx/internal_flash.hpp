#pragma once

/*
 *   Name: internal_flash.hpp
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

namespace soc {
namespace stm32l452xx {

class internal_flash
{
public:
    enum class Latency : uint32_t
    {
        _0 = FLASH_ACR_LATENCY_0WS,
        _1 = FLASH_ACR_LATENCY_1WS,
        _2 = FLASH_ACR_LATENCY_2WS,
        _3 = FLASH_ACR_LATENCY_3WS,
        _4 = FLASH_ACR_LATENCY_4WS,
        unknown
    };

    struct Cache_settings
    {
        enum class Data : uint32_t
        {
            disable = 0x0u,
            enable  = FLASH_ACR_DCEN,
            unknown
        };

        enum class Instructions : uint32_t
        {
            disable = 0x0u,
            enable  = FLASH_ACR_ICEN,
            unknown
        };

        Data data                 = Data::unknown;
        Instructions instructions = Instructions::unknown;
    };

    static void set_cache_settings(const Cache_settings& a_settings)
    {
        cml::bit_flag::set(&(FLASH->ACR), FLASH_ACR_DCEN, static_cast<uint32_t>(a_settings.data));
        cml::bit_flag::set(&(FLASH->ACR), FLASH_ACR_ICEN, static_cast<uint32_t>(a_settings.instructions));
    }

    static Cache_settings get_cache_settings()
    {
        return { static_cast<Cache_settings::Data>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_DCEN)),
                 static_cast<Cache_settings::Instructions>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_ICEN)) };
    }

    static Latency get_latency()
    {
        return static_cast<Latency>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }

private:
    internal_flash()                      = delete;
    internal_flash(const internal_flash&) = delete;
    internal_flash(internal_flash&&)      = delete;
    ~internal_flash()                     = delete;

    internal_flash& operator=(const internal_flash&) = delete;
    internal_flash& operator=(internal_flash&&) = delete;
};

} // namespace stm32l452xx
} // namespace soc