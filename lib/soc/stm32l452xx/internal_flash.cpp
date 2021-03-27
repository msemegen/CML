/*
 *   Name: internal_flash.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/internal_flash.hpp>

// soc
#include <soc/system_timer.hpp>

// cml
#include <cml/debug/assertion.hpp>

namespace soc {
namespace stm32l452xx {

internal_flash::Result
internal_flash::write_page_polling(uint32_t a_address, const void* a_p_data, uint32_t a_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_bytes > 0);

    Unlock_guard guard;

    if (true == guard.is_unlocked())
    {
    }

    return Result();
}

internal_flash::Result internal_flash::write_page_polling(uint32_t a_address,
                                                          const void* a_p_data,
                                                          uint32_t a_size_in_bytes,
                                                          uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_bytes > 0);
    cml_assert(a_timeout > 0);

    uint32_t timeout_start = system_timer::get();

    Unlock_guard guard(timeout_start, a_timeout);

    if (true == guard.is_unlocked())
    {
    }

    return Result();
}

internal_flash::Result internal_flash::read_page_polling(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_bytes > 0);

    Unlock_guard guard;

    if (true == guard.is_unlocked())
    {
    }

    return Result();
}

internal_flash::Result
internal_flash::read_page_polling(uint32_t a_address, void* a_p_data, uint32_t a_size_in_bytes, uint32_t a_timeout)
{
    cml_assert(nullptr != a_p_data);
    cml_assert(a_size_in_bytes > 0);
    cml_assert(a_timeout > 0);

    uint32_t timeout_start = system_timer::get();

    Unlock_guard guard(timeout_start, a_timeout);

    if (true == guard.is_unlocked())
    {
    }

    return Result();
}

internal_flash::Result internal_flash::erase_page_polling(uint32_t a_address)
{
    Unlock_guard guard;

    if (true == guard.is_unlocked())
    {
    }

    return Result();
}

internal_flash::Result internal_flash::erase_page_polling(uint32_t a_address, uint32_t a_timeout)
{
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