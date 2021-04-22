/*
 *   Name: internal_flash.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L011xx

// this
#include <soc/stm32l011xx/internal_flash.hpp>

// cml
#include <cml/bit_flag.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/utils/wait_until.hpp>

// std
#include <cstring>

namespace {

using namespace cml;
using namespace soc::stm32l011xx;

void clear_FLASH_SR_errors()
{
    bit_flag::set(&(FLASH->SR),
                  FLASH_SR_SIZERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR |
                      FLASH_SR_FWWERR);
}

bool is_FLASH_SR_error()
{
    return bit::is_any(FLASH->SR,
                       FLASH_SR_SIZERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR |
                           FLASH_SR_FWWERR);
}

internal_flash::Result::Status_flag get_status_flag_from_FLASH_SR()
{
    uint32_t SR = (FLASH->SR & 0x32F00u);
    return static_cast<internal_flash::Result::Status_flag>(SR);
}

} // namespace

namespace soc {
namespace stm32l011xx {

using namespace cml;
using namespace cml::utils;

internal_flash::Result
internal_flash::write_polling(uint32_t a_address, const uint32_t* a_p_data, uint32_t a_size_in_words)
{
    cml_assert(a_address >= start_address && a_address <= start_address + size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_words > 0);

    Unlock_guard guard;
    Result ret { Result::Status_flag::locked, 0 };

    if (true == guard.is_unlocked())
    {
        clear_FLASH_SR_errors();

        volatile uint32_t* p_address = reinterpret_cast<volatile uint32_t*>(a_address);
        for (; ret.words < a_size_in_words && false == is_FLASH_SR_error(); ret.words++)
        {
            if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
            {
                bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
            }

            (*p_address) = a_p_data[ret.words];

            wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);
        }

        ret.status = get_status_flag_from_FLASH_SR();

        if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
        {
            bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
        }
    }

    return ret;
}

internal_flash::Result internal_flash::write_polling(uint32_t a_address,
                                                     const uint32_t* a_p_data,
                                                     uint32_t a_size_in_words,
                                                     uint32_t a_timeout)
{
    cml_assert(a_address >= start_address && a_address <= start_address + size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_words > 0);
    cml_assert(a_timeout > 0);

    uint32_t timeout_start = system_timer::get();

    Unlock_guard guard(timeout_start, a_timeout);
    Result ret { Result::Status_flag::locked, 0 };

    if (true == guard.is_unlocked())
    {
        clear_FLASH_SR_errors();

        volatile uint32_t* p_address = reinterpret_cast<volatile uint32_t*>(a_address);
        for (; ret.words < a_size_in_words && false == is_FLASH_SR_error() &&
               a_timeout >= various::time_diff(system_timer::get(), timeout_start);
             ret.words++)
        {
            if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
            {
                bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
            }

            *(p_address + ret.words) = a_p_data[ret.words];

            wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);
        }

        ret.status = get_status_flag_from_FLASH_SR();

        if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
        {
            bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
        }
    }

    return ret;
}

internal_flash::Result internal_flash::read_polling(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes)
{
    cml_assert(a_address >= start_address && a_address <= start_address + size_in_bytes);
    cml_assert(a_address <= a_address + page_size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_bytes > 0 && a_size_in_bytes <= page_size_in_bytes);

    Unlock_guard guard;

    if (true == guard.is_unlocked())
    {
        std::memcpy(a_p_data, reinterpret_cast<const uint8_t*>(a_address), a_size_in_bytes);
    }

    return { get_status_flag_from_FLASH_SR(), 0 };
}

internal_flash::Result
internal_flash::read_polling(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes, uint32_t a_timeout)
{
    cml_assert(a_address >= start_address && a_address <= start_address + size_in_bytes);
    cml_assert(a_address <= a_address + page_size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_bytes > 0 && a_size_in_bytes <= page_size_in_bytes);
    cml_assert(a_timeout > 0);

    uint32_t timeout_start = system_timer::get();

    Unlock_guard guard(timeout_start, a_timeout);
    Result ret { Result::Status_flag::locked, 0 };

    if (true == guard.is_unlocked())
    {
        std::memcpy(a_p_data, reinterpret_cast<const uint8_t*>(a_address), a_size_in_bytes);
        ret.status = get_status_flag_from_FLASH_SR();
    }

    return ret;
}

} // namespace stm32l011xx
} // namespace soc

#endif //  STM32L011xx