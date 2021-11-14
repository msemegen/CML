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
#include <stm32l4xx.h>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/hal/Interrupt_guard.hpp>
#include <cml/utils/wait_until.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class internal_flash : private cml::Non_constructible
{
public:
    enum
    {
        start_address      = FLASH_BASE,
        page_size_in_bytes = 2048u,
        size_in_bytes      = page_size_in_bytes * 256
    };

    enum class Latency : uint32_t
    {
        _0 = FLASH_ACR_LATENCY_0WS,
        _1 = FLASH_ACR_LATENCY_1WS,
        _2 = FLASH_ACR_LATENCY_2WS,
        _3 = FLASH_ACR_LATENCY_3WS,
        _4 = FLASH_ACR_LATENCY_4WS,
    };

    enum class Cache_mode_flag : uint32_t
    {
        disabled     = 0x0u,
        data         = FLASH_ACR_DCEN,
        instructions = FLASH_ACR_ICEN,
        prefetech    = FLASH_ACR_PRFTEN
    };

    static void set_cache_mode(Cache_mode_flag a_mode)
    {
        cml::bit_flag::set(
            &(FLASH->ACR), FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN, static_cast<uint32_t>(a_mode));
    }

    static Cache_mode_flag get_cache_mode()
    {
        return static_cast<Cache_mode_flag>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_DCEN | FLASH_ACR_ICEN));
    }

    static void set_latency(Latency a_latency)
    {
        cml::bit_flag::set(&(FLASH->ACR), FLASH_ACR_LATENCY, static_cast<uint32_t>(a_latency));
        cml::utils::wait_until::all_bits(&(FLASH->ACR), static_cast<uint32_t>(a_latency), false);
    }

    static Latency get_latency()
    {
        return static_cast<Latency>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }
};

constexpr internal_flash::Cache_mode_flag operator|(internal_flash::Cache_mode_flag a_f1,
                                                    internal_flash::Cache_mode_flag a_f2)
{
    return static_cast<internal_flash::Cache_mode_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr internal_flash::Cache_mode_flag operator&(internal_flash::Cache_mode_flag a_f1,
                                                    internal_flash::Cache_mode_flag a_f2)
{
    return static_cast<internal_flash::Cache_mode_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr internal_flash::Cache_mode_flag operator|=(internal_flash::Cache_mode_flag& a_f1,
                                                     internal_flash::Cache_mode_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc