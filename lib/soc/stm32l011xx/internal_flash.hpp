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
#include <stm32l011xx.h>

// cml
#include <cml/bit_flag.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l011xx {

class internal_flash
{
public:
    enum class Latency : uint32_t
    {
        _0 = 0x0u,
        _1 = FLASH_ACR_LATENCY,
        unknown
    };

public:
    static Latency get_latency()
    {
        return static_cast<Latency>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }

private:
    class Lock_guard : public cml::Non_copyable
    {
    public:

        Lock_guard() {}
        ~Lock_guard() {}
    };

private:
    internal_flash()                      = delete;
    internal_flash(const internal_flash&) = delete;
    internal_flash(internal_flash&&)      = delete;
    ~internal_flash()                     = delete;

    internal_flash& operator=(const internal_flash&) = delete;
    internal_flash& operator=(internal_flash&&) = delete;
};

} // namespace stm32l011xx
} // namespace soc
