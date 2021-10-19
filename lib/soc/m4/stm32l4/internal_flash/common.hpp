#pragma once

/*
 *   Name: common.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// externals
#include <stm32l4xx.h>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/internal_flash/internal_flash.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/utils/wait_until.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class Unlock_guard : private cml::Non_copyable
{
public:
    Unlock_guard()
    {
        cml::utils::wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

        if (true == cml::bit_flag::is(FLASH->CR, FLASH_CR_LOCK))
        {
            Interrupt_guard interrupt_guard;

            FLASH->KEYR = 0x45670123u;
            FLASH->KEYR = 0xCDEF89ABu;
        }

        this->unlocked = false == cml::bit_flag::is(FLASH->CR, FLASH_CR_LOCK);
    }

    Unlock_guard(uint32_t a_start, uint32_t a_timeout)
    {
        this->unlocked = cml::utils::wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true, a_start, a_timeout);

        if (true == this->unlocked && true == cml::bit_flag::is(FLASH->CR, FLASH_CR_LOCK))
        {
            Interrupt_guard interrupt_guard;

            FLASH->KEYR = 0x45670123u;
            FLASH->KEYR = 0xCDEF89ABu;
        }

        this->unlocked = false == cml::bit_flag::is(FLASH->CR, FLASH_CR_LOCK);
    }

    ~Unlock_guard()
    {
        cml::bit_flag::set(&(FLASH->CR), FLASH_CR_LOCK);
        this->unlocked = false == cml::bit_flag::is(FLASH->CR, FLASH_CR_LOCK);
    }

    bool is_unlocked() const
    {
        return this->unlocked;
    }

private:
    bool unlocked;
};

class Cache_disabler_guard : private cml::Non_copyable
{
public:
    Cache_disabler_guard()
        : cache_mode(internal_flash::get_cache_mode())
    {
        internal_flash::set_cache_mode(internal_flash::Cache_mode_flag::disabled);
    }

    ~Cache_disabler_guard()
    {
        internal_flash::set_cache_mode(this->cache_mode);
    }

private:
    const internal_flash::Cache_mode_flag cache_mode;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc