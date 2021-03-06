#pragma once

/*
 *   Name: RNG.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// external
#include <stm32l452xx.h>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l452xx {
namespace peripherals {

#ifdef RNG
#undef RNG
#endif

class RNG : private cml::Non_copyable
{
public:
    struct New_value_callback
    {
        using Function = void (*)(uint32_t a_value, bool a_clock_error, bool a_seed_error, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    RNG();
    ~RNG();

    bool enable(uint32_t a_irq_priority, uint32_t a_timeout);
    void disable();

    bool get_value_polling(uint32_t* a_p_value, uint32_t a_timeout);

    void register_new_value_callback(const New_value_callback& a_callback);
    void unregister_new_value_callback();
};

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc