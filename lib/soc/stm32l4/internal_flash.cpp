/*
 *   Name: internal_flash.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// std
#include <cstring>

// this
#include <soc/stm32l4/internal_flash.hpp>

// soc
#include <soc/stm32l4/mcu.hpp>
#include <soc/system_timer.hpp>

// cml
#include <cml/debug/assertion.hpp>

namespace {
using namespace cml;
using namespace soc::stm32l4;

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

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
    uint32_t SR = (FLASH->SR & 0x3F8u);
    return static_cast<internal_flash::Result::Status_flag>(SR);
}

#endif

} // namespace

namespace soc {
namespace stm32l4 {

using namespace cml;
using namespace cml::utils;

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

internal_flash::Result internal_flash::write_polling(uint32_t a_address,
                                                     const uint64_t* a_p_data,
                                                     uint32_t a_size_in_double_words,
                                                     Mode a_mode)
{
    cml_assert(a_address >= start_address && a_address <= start_address + size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_double_words > 0);
    cml_assert((Mode::fast == a_mode && mcu::Voltage_scaling::_1 == mcu::get_voltage_scaling()) ||
               Mode::standard == a_mode);

    Unlock_guard guard;
    Result ret { Result::Status_flag::locked, 0 };

    if (true == guard.is_unlocked())
    {
        Cache_disabler_guard cache_disabler_guard;
        clear_FLASH_SR_errors();

        switch (a_mode)
        {
            case internal_flash::Mode::standard: {
                bit_flag::set(&(FLASH->CR), FLASH_CR_PG);

                volatile uint32_t* p_address = reinterpret_cast<volatile uint32_t*>(a_address);
                for (; ret.words < a_size_in_double_words && false == is_FLASH_SR_error(); ret.words++)
                {
                    *(p_address + ret.words * 2u + 0u) = static_cast<uint32_t>(a_p_data[ret.words] >> 0x00u);
                    *(p_address + ret.words * 2u + 1u) = static_cast<uint32_t>(a_p_data[ret.words] >> 0x20u);

                    wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

                    if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
                    {
                        bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
                    }
                }

                ret.status = get_status_flag_from_FLASH_SR();
                bit_flag::clear(&(FLASH->CR), FLASH_CR_PG);
            }
            break;
            case internal_flash::Mode::fast: {
                bit_flag::set(&(FLASH->CR), FLASH_CR_FSTPG);

                volatile uint32_t* p_address = reinterpret_cast<volatile uint32_t*>(a_address);
                for (; ret.words < a_size_in_double_words && false == is_FLASH_SR_error(); ret.words++)
                {
                    *(p_address + ret.words * 2u + 0u) = static_cast<uint32_t>(a_p_data[ret.words] >> 0x00u);
                    *(p_address + ret.words * 2u + 1u) = static_cast<uint32_t>(a_p_data[ret.words] >> 0x20u);

                    wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

                    if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
                    {
                        bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
                    }
                }

                ret.status = get_status_flag_from_FLASH_SR();
                bit_flag::clear(&(FLASH->CR), FLASH_CR_FSTPG);
            }
            break;
        }
    }

    return ret;
}

internal_flash::Result internal_flash::write_polling(uint32_t a_address,
                                                     const uint64_t* a_p_data,
                                                     uint32_t a_size_in_double_words,
                                                     Mode a_mode,
                                                     uint32_t a_timeout)
{
    cml_assert(a_address >= start_address && a_address <= start_address + size_in_bytes);
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_double_words > 0);
    cml_assert((Mode::fast == a_mode && mcu::Voltage_scaling::_1 == mcu::get_voltage_scaling()) ||
               Mode::standard == a_mode);
    cml_assert(a_timeout > 0);

    uint32_t timeout_start = system_timer::get();

    Unlock_guard guard(timeout_start, a_timeout);
    Result ret { Result::Status_flag::locked, 0 };

    if (true == guard.is_unlocked())
    {
        Cache_disabler_guard cache_disabler_guard;
        clear_FLASH_SR_errors();

        switch (a_mode)
        {
            case internal_flash::Mode::standard: {
                bit_flag::set(&(FLASH->CR), FLASH_CR_PG);

                volatile uint32_t* p_address = reinterpret_cast<volatile uint32_t*>(a_address);
                for (; ret.words < a_size_in_double_words && false == is_FLASH_SR_error() &&
                       a_timeout >= various::time_diff(system_timer::get(), timeout_start);
                     ret.words++)
                {
                    *(p_address + ret.words * 2u + 0u) = static_cast<uint32_t>(a_p_data[ret.words] >> 0x00u);
                    *(p_address + ret.words * 2u + 1u) = static_cast<uint32_t>(a_p_data[ret.words] >> 0x20u);

                    wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

                    if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
                    {
                        bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
                    }
                }

                ret.status = get_status_flag_from_FLASH_SR();
                bit_flag::clear(&(FLASH->CR), FLASH_CR_PG);
            }
            break;

            case internal_flash::Mode::fast: {
                bit_flag::set(&(FLASH->CR), FLASH_CR_FSTPG);

                volatile uint32_t* p_address = reinterpret_cast<volatile uint32_t*>(a_address);
                for (; ret.words < a_size_in_double_words && false == is_FLASH_SR_error() &&
                       a_timeout >= various::time_diff(system_timer::get(), timeout_start);
                     ret.words++)
                {
                    *(p_address + ret.words * 2u + 0u) = static_cast<uint32_t>(a_p_data[ret.words] >> 0x00u);
                    *(p_address + ret.words * 2u + 1u) = static_cast<uint32_t>(a_p_data[ret.words] >> 0x20u);

                    wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

                    if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
                    {
                        bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
                    }
                }

                ret.status = get_status_flag_from_FLASH_SR();
                bit_flag::clear(&(FLASH->CR), FLASH_CR_FSTPG);
            }
            break;
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

internal_flash::Result internal_flash::erase_page_polling(uint32_t a_page_address)
{
    cml_assert(a_page_address >= start_address && a_page_address <= start_address + size_in_bytes);

    Unlock_guard unlock_guard;

    if (true == unlock_guard.is_unlocked())
    {
        Cache_disabler_guard cache_disabler_guard;
        clear_FLASH_SR_errors();

        uint32_t index = (a_page_address - start_address) / page_size_in_bytes;

        bit_flag::set(&(FLASH->CR), FLASH_CR_PNB, (index << FLASH_CR_PNB_Pos));
        bit_flag::set(&(FLASH->CR), FLASH_CR_PER | FLASH_CR_STRT);
        wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);

        bit_flag::clear(&(FLASH->CR), FLASH_CR_PER | FLASH_CR_STRT);
    }

    return { get_status_flag_from_FLASH_SR(), 0 };
}

internal_flash::Result internal_flash::erase_page_polling(uint32_t a_page_address, uint32_t a_timeout)
{
    cml_assert(a_page_address >= start_address && a_page_address <= start_address + size_in_bytes);
    cml_assert(a_timeout > 0);

    uint32_t timeout_start = system_timer::get();
    Result ret { Result::Status_flag::locked, 0 };

    Unlock_guard guard(timeout_start, a_timeout);

    if (true == guard.is_unlocked())
    {
        Cache_disabler_guard cache_disabler_guard;
        clear_FLASH_SR_errors();

        uint32_t index = (a_page_address - start_address) / page_size_in_bytes;

        bit_flag::set(&(FLASH->CR), FLASH_CR_PNB, (index << FLASH_CR_PNB_Pos));
        bit_flag::set(&(FLASH->CR), FLASH_CR_PER | FLASH_CR_STRT);

        if (true == wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true, timeout_start, a_timeout))
        {
            bit_flag::clear(&(FLASH->CR), FLASH_CR_PER | FLASH_CR_STRT);
        }

        ret.status = get_status_flag_from_FLASH_SR();
    }

    return ret;
}

internal_flash::Result internal_flash::erase_bank_polling(Bank_id)
{
    Unlock_guard unlock_guard;

    if (true == unlock_guard.is_unlocked())
    {
        Cache_disabler_guard cache_disabler_guard;
        clear_FLASH_SR_errors();

        bit_flag::set(&(FLASH->CR), FLASH_CR_MER1 | FLASH_CR_STRT);
        wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true);
    }

    return { get_status_flag_from_FLASH_SR(), 0 };
}

internal_flash::Result internal_flash::erase_bank_polling(Bank_id, uint32_t a_timeout)
{
    cml_assert(a_timeout > 0);

    uint32_t timeout_start = system_timer::get();
    Result ret { Result::Status_flag::locked, 0 };

    Unlock_guard unlock_guard(timeout_start, a_timeout);

    if (true == unlock_guard.is_unlocked())
    {
        Cache_disabler_guard cache_disabler_guard;
        clear_FLASH_SR_errors();

        bit_flag::set(&(FLASH->CR), FLASH_CR_MER1 | FLASH_CR_STRT);
        wait_until::all_bits(&(FLASH->SR), FLASH_SR_BSY, true, timeout_start, a_timeout);

        ret.status = get_status_flag_from_FLASH_SR();
    }

    return ret;
}

#endif

} // namespace stm32l4
} // namespace soc

#endif // STM32L4