/*
 *   Name: internal_flash.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx

// std
#include <cstring>

// this
#include <soc/stm32l452xx/internal_flash.hpp>

// soc
#include <soc/stm32l452xx/mcu.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/debug/assertion.hpp>

namespace {
using namespace cml;
using namespace soc::stm32l452xx;

void clear_FLASH_SR_errors()
{
    bit_flag::set(&(FLASH->SR),
                  FLASH_SR_PROGERR | FLASH_SR_SIZERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR | FLASH_SR_WRPERR |
                      FLASH_SR_MISERR | FLASH_SR_FASTERR);
}

bool is_FLASH_SR_error()
{
    return bit::is_any(FLASH->SR,
                       FLASH_SR_PROGERR | FLASH_SR_SIZERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR | FLASH_SR_WRPERR |
                           FLASH_SR_MISERR | FLASH_SR_FASTERR);
}

internal_flash::Result::Status_flag get_status_flag_from_FLASH_SR()
{
    uint32_t SR = (FLASH->SR & 0x3F8);
    return static_cast<internal_flash::Result::Status_flag>(SR);
}

} // namespace

namespace soc {
namespace stm32l452xx {

using namespace cml;
using namespace cml::utils;

internal_flash::Result internal_flash::write_polling(uint32_t a_page_address,
                                                     const uint64_t* a_p_data,
                                                     uint32_t a_size_in_double_words,
                                                     Mode a_mode)
{
    cml_assert(a_page_address >= start_address && a_page_address <= start_address + size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_double_words > 0 && a_size_in_double_words <= page_size_in_bytes / 8u);
    cml_assert((Mode::fast == a_mode && mcu::Voltage_scaling::_1 == mcu::get_voltage_scaling()) ||
               Mode::standard == a_mode);

    Unlock_guard guard;
    uint32_t words = 0;

    if (true == guard.is_unlocked())
    {
        const Cache_mode_flag cache_mode = get_cache_mode();
        bool is_error                    = false;

        set_cache_mode(Cache_mode_flag::disabled);

        switch (a_mode)
        {
            case internal_flash::Mode::standard: {
                clear_FLASH_SR_errors();

                for (; words < a_size_in_double_words && false == is_error; words++)
                {
                    bit_flag::set(&(FLASH->CR), FLASH_CR_PG);

                    volatile uint32_t* p_address = reinterpret_cast<volatile uint32_t*>(a_page_address);

                    *(p_address + 0) = static_cast<uint32_t>(a_p_data[words] >> 0x00u);
                    *(p_address + 1) = static_cast<uint32_t>(a_p_data[words] >> 0x20u);

                    wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);
                    bit_flag::clear(&(FLASH->CR), FLASH_CR_PG);

                    is_error = is_FLASH_SR_error();
                }
            }
            break;
            case internal_flash::Mode::fast: {
            }
            break;
        }

        set_cache_mode(cache_mode);
    }

    return { get_status_flag_from_FLASH_SR(), words };
}

internal_flash::Result internal_flash::write_polling(uint32_t a_page_address,
                                                     const uint64_t* a_p_data,
                                                     uint32_t a_size_in_double_words,
                                                     Mode a_mode,
                                                     uint32_t a_timeout)
{
    cml_assert(a_page_address >= start_address && a_page_address <= start_address + size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_double_words > 0 && a_size_in_double_words <= page_size_in_bytes / 8u);
    cml_assert((Mode::fast == a_mode && mcu::Voltage_scaling::_1 == mcu::get_voltage_scaling()) ||
               Mode::standard == a_mode);
    cml_assert(a_timeout > 0);

    uint32_t timeout_start = system_timer::get();

    Unlock_guard guard(timeout_start, a_timeout);

    if (true == guard.is_unlocked())
    {
        const Cache_mode_flag cache_mode = get_cache_mode();
        set_cache_mode(Cache_mode_flag::disabled);

        bool is_error = false;

        switch (a_mode)
        {
            case internal_flash::Mode::standard: {
            }
            break;
            case internal_flash::Mode::fast: {
            }
            break;
        }

        set_cache_mode(cache_mode);
    }

    return Result();
}

internal_flash::Result
internal_flash::read_polling(uint32_t a_page_address, uint32_t a_offset, void* a_p_data, uint32_t a_size_in_bytes)
{
    cml_assert(a_page_address >= start_address && a_page_address <= start_address + size_in_bytes);
    cml_assert(a_page_address + a_offset <= a_page_address + page_size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_bytes > 0 && a_size_in_bytes <= page_size_in_bytes);

    Unlock_guard guard;

    if (true == guard.is_unlocked())
    {
        std::memcpy(a_p_data, reinterpret_cast<const uint8_t*>(a_page_address + a_offset), a_size_in_bytes);
    }

    return { get_status_flag_from_FLASH_SR(), a_size_in_bytes };
}

internal_flash::Result internal_flash::read_polling(uint32_t a_page_address,
                                                    uint32_t a_offset,
                                                    void* a_p_data,
                                                    uint32_t a_size_in_bytes,
                                                    uint32_t a_timeout)
{
    cml_assert(a_page_address >= start_address && a_page_address <= start_address + size_in_bytes);
    cml_assert(a_page_address + a_offset <= a_page_address + page_size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_bytes > 0 && a_size_in_bytes <= page_size_in_bytes);
    cml_assert(a_timeout > 0);

    uint32_t timeout_start = system_timer::get();

    Unlock_guard guard(timeout_start, a_timeout);

    if (true == guard.is_unlocked())
    {
        std::memcpy(a_p_data, reinterpret_cast<const uint8_t*>(a_page_address + a_offset), a_size_in_bytes);
    }

    return Result();
}

internal_flash::Result internal_flash::erase_page_polling(uint32_t a_page_address, Mode a_mode)
{
    cml_assert(a_page_address >= start_address && a_page_address <= start_address + size_in_bytes);
    cml_assert((Mode::fast == a_mode && mcu::Voltage_scaling::_1 == mcu::get_voltage_scaling()) ||
               Mode::standard == a_mode);

    Unlock_guard guard;

    if (true == guard.is_unlocked())
    {
        const Cache_mode_flag cache_mode = get_cache_mode();
        set_cache_mode(Cache_mode_flag::disabled);

        bool is_error = false;

        switch (a_mode)
        {
            case internal_flash::Mode::standard: {
                clear_FLASH_SR_errors();

                uint32_t index = (a_page_address - start_address) / page_size_in_bytes;

                bit_flag::set(&(FLASH->CR), FLASH_CR_PNB, (index << FLASH_CR_PNB_Pos));
                bit_flag::set(&(FLASH->CR), FLASH_CR_PER | FLASH_CR_STRT);
                wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

                bit_flag::clear(&(FLASH->CR), FLASH_CR_PER | FLASH_CR_STRT);
            }
            break;
            case internal_flash::Mode::fast: {
            }
            break;
        }

        set_cache_mode(cache_mode);
    }

    return { get_status_flag_from_FLASH_SR(), 0 };
}

internal_flash::Result internal_flash::erase_page_polling(uint32_t a_page_address, Mode a_mode, uint32_t a_timeout)
{
    cml_assert(a_page_address >= start_address && a_page_address <= start_address + size_in_bytes);
    cml_assert((Mode::fast == a_mode && mcu::Voltage_scaling::_1 == mcu::get_voltage_scaling()) ||
               Mode::standard == a_mode);
    cml_assert(a_timeout > 0);

    Unlock_guard guard;

    if (true == guard.is_unlocked())
    {
    }

    return Result();
}

} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx