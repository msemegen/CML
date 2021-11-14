#pragma once

/*
 *   Name: Polling.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// soc
#include <soc/m4/stm32l4/Polling.hpp>
#include <soc/m4/stm32l4/internal_flash/internal_flash.hpp>

// cml
#include <cml/Non_constructible.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
template<> class Polling<internal_flash> : private cml::Non_constructible
{
public:
    enum class Mode
    {
        standard,
        fast
    };

    enum class Bank_id
    {
        _0
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
        };

        Status_flag status = cml::various::get_enum_incorrect_value<Status_flag>();
        uint32_t words     = 0;
    };

    static Result write(uint32_t a_address, const uint64_t* a_p_data, uint32_t a_size_in_double_words, Mode a_mode);
    static Result write(uint32_t a_address,
                        const uint64_t* a_p_data,
                        uint32_t a_size_in_double_words,
                        Mode a_mode,
                        uint32_t a_timeout);

    static Result read(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes);
    static Result read(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes, uint32_t a_timeout);

    static Result erase_page(uint32_t a_page_address);
    static Result erase_page(uint32_t a_page_address, uint32_t a_timeout);

    static Result erase_bank(Bank_id a_id);
    static Result erase_bank(Bank_id a_id, uint32_t a_timeout);
};
} // namespace stm32l4
} // namespace m4
} // namespace soc