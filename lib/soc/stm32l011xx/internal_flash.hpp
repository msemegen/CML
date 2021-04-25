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

// soc
#include <soc/Interrupt_guard.hpp>

namespace soc {
namespace stm32l011xx {

class internal_flash
{
public:
    enum
    {
        start_address      = FLASH_BASE,
        page_size_in_bytes = 128,
        size_in_bytes      = page_size_in_bytes * 128
    };

    enum class Latency : uint32_t
    {
        _0 = 0x0u,
        _1 = FLASH_ACR_LATENCY,
        unknown
    };

    struct Result
    {
        enum class Status_flag : uint32_t
        {
            ok                     = 0x0u,
            size_error             = FLASH_SR_SIZERR,
            page_alignment_error   = FLASH_SR_PGAERR,
            write_protection_error = FLASH_SR_WRPERR,
            read_protection_error  = FLASH_SR_RDERR,
            not_zero_error         = FLASH_SR_NOTZEROERR,
            operation_abroted      = FLASH_SR_FWWER,
            locked,
            unknown
        };

        Status_flag status = Status_flag::unknown;
        uint32_t words     = 0;
    };

public:
    static Latency get_latency()
    {
        return static_cast<Latency>(cml::bit_flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }

    static Result write_polling(uint32_t a_address, const uint32_t* a_p_data, uint32_t a_size_in_words);
    static Result
    write_polling(uint32_t a_address, const uint32_t* a_p_data, uint32_t a_size_in_words, uint32_t a_timeout);

    static Result read_polling(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes);
    static Result read_polling(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes, uint32_t a_timeout);

    static Result erase_page_polling(uint32_t a_page_address, uint32_t a_size_in_words);
    static Result erase_page_polling(uint32_t a_page_address, uint32_t a_size_in_words, uint32_t a_timeout);

private:
    class Unlock_guard : public cml::Non_copyable
    {
    public:
        Unlock_guard()
        {
            cml::utils::wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

            if (true == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
            {
                Interrupt_guard guard;

                FLASH->PEKEYR = 0x89ABCDEFu;
                FLASH->PEKEYR = 0x02030405u;
            }

            if (true == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PRGLOCK))
            {
                Interrupt_guard guard;

                FLASH->PRGKEYR = 0x8C9DAEBFu;
                FLASH->PRGKEYR = 0x13141516u;
            }

            this->unlocked = false == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK) &&
                             false == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PRGLOCK);
        }

        Unlock_guard(uint32_t a_start, uint32_t a_timeout)
        {
            this->unlocked = cml::utils::wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true, a_start, a_timeout);

            if (true == this->unlocked)
            {
                if (true == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
                {
                    Interrupt_guard guard;

                    FLASH->PEKEYR = 0x89ABCDEFu;
                    FLASH->PEKEYR = 0x02030405u;
                }

                if (true == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PRGLOCK))
                {
                    Interrupt_guard guard;

                    FLASH->PRGKEYR = 0x8C9DAEBFu;
                    FLASH->PRGKEYR = 0x13141516u;
                }

                this->unlocked = false == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK) &&
                                 false == cml::bit_flag::is(FLASH->PECR, FLASH_PECR_PRGLOCK);
            }
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
