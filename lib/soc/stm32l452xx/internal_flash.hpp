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
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>

namespace soc {
namespace stm32l452xx {

class internal_flash
{
public:
    enum
    {
        page_size_in_bytes = 2048
    };

    enum class Latency : uint32_t
    {
        _0 = FLASH_ACR_LATENCY_0WS,
        _1 = FLASH_ACR_LATENCY_1WS,
        _2 = FLASH_ACR_LATENCY_2WS,
        _3 = FLASH_ACR_LATENCY_3WS,
        _4 = FLASH_ACR_LATENCY_4WS,
        unknown
    };

    enum class Mode
    {
        standard,
        fast
    };

    enum class Cache_settings_flag : uint32_t
    {
        data         = FLASH_ACR_DCEN,
        instructions = FLASH_ACR_ICEN
    };

    struct Result
    {
    };

    static void set_cache_settings(const Cache_settings_flag& a_settings)
    {
        cml::bit_flag::set(&(FLASH->ACR), FLASH_ACR_DCEN | FLASH_ACR_ICEN, static_cast<uint32_t>(a_settings));
    }

    void set_prefetch_settings(bool a_enable)
    {
        if (true == a_enable)
        {
            cml::bit_flag::set(&(FLASH->ACR), FLASH_ACR_PRFTEN);
        }
        else
        {
            cml::bit_flag::clear(&(FLASH->ACR), FLASH_ACR_PRFTEN);
        }
    }

    static Cache_settings_flag get_cache_settings()
    {
        return static_cast<Cache_settings_flag>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_DCEN | FLASH_ACR_ICEN));
    }

    static Latency get_latency()
    {
        return static_cast<Latency>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }

    static Result write_polling(uint32_t a_address, const void* a_p_data, uint32_t a_size_in_bytes, Mode a_mode);
    static Result
    write_polling(uint32_t a_address, const void* a_p_data, uint32_t a_size_in_bytes, Mode a_mode, uint32_t a_timeout);

    static Result read_polling(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes);
    static Result read_polling(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes, uint32_t a_timeout);

    static Result erase_page_polling(uint32_t a_address, Mode a_mode);
    static Result erase_page_polling(uint32_t a_address, Mode a_mode, uint32_t a_timeout);

private:
    class Unlock_guard : public cml::Non_copyable
    {
    public:
        Unlock_guard()
        {
            cml::utils::wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

            if (true == cml::bit::is(FLASH->CR, FLASH_CR_LOCK))
            {
                FLASH->KEYR = 0x45670123u;
                FLASH->KEYR = 0xCDEF89ABu;
            }

            this->unlocked = true;
        }

        Unlock_guard(uint32_t a_start, uint32_t a_timeout)
        {
            this->unlocked = cml::utils::wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true, a_start, a_timeout);

            if (true == this->unlocked && true == cml::bit::is(FLASH->CR, FLASH_CR_LOCK))
            {
                FLASH->KEYR = 0x45670123u;
                FLASH->KEYR = 0xCDEF89ABu;
            }
        }

        ~Unlock_guard()
        {
            cml::bit_flag::set(&(FLASH->CR), FLASH_CR_LOCK);
            this->unlocked = false;
        }

        bool is_unlocked() const
        {
            return this->unlocked;
        }

    private:
        bool unlocked;
    };

private:
    internal_flash()                      = delete;
    internal_flash(const internal_flash&) = delete;
    internal_flash(internal_flash&&)      = delete;
    ~internal_flash()                     = delete;

    internal_flash& operator=(const internal_flash&) = delete;
    internal_flash& operator=(internal_flash&&) = delete;
};

constexpr internal_flash::Cache_settings_flag operator|(internal_flash::Cache_settings_flag a_f1,
                                                        internal_flash::Cache_settings_flag a_f2)
{
    return static_cast<internal_flash::Cache_settings_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr internal_flash::Cache_settings_flag operator&(internal_flash::Cache_settings_flag a_f1,
                                                        internal_flash::Cache_settings_flag a_f2)
{
    return static_cast<internal_flash::Cache_settings_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr internal_flash::Cache_settings_flag operator|=(internal_flash::Cache_settings_flag& a_f1,
                                                         internal_flash::Cache_settings_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}


} // namespace stm32l452xx
} // namespace soc