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
#include <cml/Non_copyable.hpp>
#include <cml/bit_flag.hpp>
#include <cml/utils/wait_until.hpp>

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

    enum class Bank_id
    {
        _0
    };

    struct Result
    {
    };

public:
    static Latency get_latency()
    {
        return static_cast<Latency>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }

    static Result write_polling(uint32_t a_page_address, const uint64_t* a_p_data, uint32_t a_size_in_double_words);
    static Result write_polling(uint32_t a_page_address,
                                const uint64_t* a_p_data,
                                uint32_t a_size_in_double_words,
                                uint32_t a_timeout);

    static Result read_polling(uint32_t a_page_address, uint32_t a_offset, void* a_p_data, uint32_t a_size_in_bytes);
    static Result read_polling(uint32_t a_page_address,
                               uint32_t a_offset,
                               void* a_p_data,
                               uint32_t a_size_in_bytes,
                               uint32_t a_timeout);

    static Result erase_page_polling(uint32_t a_page_address);
    static Result erase_page_polling(uint32_t a_page_address, uint32_t a_timeout);

    static Result erase_bank_polling(Bank_id a_id);
    static Result erase_bank_polling(Bank_id a_id, uint32_t a_timeout);

private:
    class Unlock_guard : public cml::Non_copyable
    {
    public:
        Unlock_guard()
        {
            cml::utils::wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

            if (true == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
            {
                FLASH->PEKEYR = 0x89ABCDEFu;
                FLASH->PEKEYR = 0x02030405u;
            }

            this->unlocked = false == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK);
        }

        Unlock_guard(uint32_t a_start, uint32_t a_timeout)
        {
            this->unlocked = cml::utils::wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true, a_start, a_timeout);

            if (true == this->unlocked && true == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
            {
                FLASH->PEKEYR = 0x89ABCDEFu;
                FLASH->PEKEYR = 0x02030405u;
            }

            this->unlocked = false == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK);
        }

        ~Unlock_guard()
        {
            cml::bit_flag::set(&(FLASH->PECR), FLASH_PECR_PELOCK);
            this->unlocked = false == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK);
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

} // namespace stm32l011xx
} // namespace soc
