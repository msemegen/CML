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
        unknown
    };

    enum class Mode
    {
        standard,
        fast
    };

    enum class Cache_mode_flag : uint32_t
    {
        disabled     = 0x0u,
        data         = FLASH_ACR_DCEN,
        instructions = FLASH_ACR_ICEN
    };

    struct Result
    {
        enum class Status_flag : uint32_t
        {
            ok                          = 0x0u,
            fast_programming_miss_error = FLASH_SR_MISERR,
            programming_sequence_error  = FLASH_SR_PGSERR,
            size_error                  = FLASH_SR_SIZERR,
            page_alignment_error        = FLASH_SR_PGAERR,
            write_protection_error      = FLASH_SR_WRPERR,
            fast_programming_error      = FLASH_SR_FASTERR,
            programming_error           = FLASH_SR_PROGERR,
            locked,
            unknown
        };

        Status_flag status = Status_flag::unknown;
        uint32_t words     = 0;
    };

    static void set_cache_mode(const Cache_mode_flag& a_mode)
    {
        cml::bit_flag::set(&(FLASH->ACR), FLASH_ACR_DCEN | FLASH_ACR_ICEN, static_cast<uint32_t>(a_mode));
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

    static Cache_mode_flag get_cache_mode()
    {
        return static_cast<Cache_mode_flag>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_DCEN | FLASH_ACR_ICEN));
    }

    static Latency get_latency()
    {
        return static_cast<Latency>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }

    static Result
    write_polling(uint32_t a_page_address, const uint64_t* a_p_data, uint32_t a_size_in_double_words, Mode a_mode);
    static Result write_polling(uint32_t a_page_address,
                                const uint64_t* a_p_data,
                                uint32_t a_size_in_double_words,
                                Mode a_mode,
                                uint32_t a_timeout);

    static Result read_polling(uint32_t a_page_address, uint32_t a_offset, void* a_p_data, uint32_t a_size_in_bytes);
    static Result read_polling(uint32_t a_page_address,
                               uint32_t a_offset,
                               void* a_p_data,
                               uint32_t a_size_in_bytes,
                               uint32_t a_timeout);

    static Result erase_page_polling(uint32_t a_page_address, Mode a_mode);
    static Result erase_page_polling(uint32_t a_page_address, Mode a_mode, uint32_t a_timeout);

private:
    class Unlock_guard : public cml::Non_copyable
    {
    public:
        Unlock_guard()
        {
            cml::utils::wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

            if (true == cml::bit_flag::is(FLASH->CR, FLASH_CR_LOCK))
            {
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
                FLASH->KEYR = 0x45670123u;
                FLASH->KEYR = 0xCDEF89ABu;
            }
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

private:
    internal_flash()                      = delete;
    internal_flash(const internal_flash&) = delete;
    internal_flash(internal_flash&&)      = delete;
    ~internal_flash()                     = delete;

    internal_flash& operator=(const internal_flash&) = delete;
    internal_flash& operator=(internal_flash&&) = delete;
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

} // namespace stm32l452xx
} // namespace soc