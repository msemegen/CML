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
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
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
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {

template<> class rcc<peripherals::RNG> : private cml::Non_constructible
{
public:
    static void enable(bool a_enable_in_lp);
    static void disable();

private:
    rcc()           = delete;
    rcc(const rcc&) = delete;
    rcc(rcc&&)      = delete;
    ~rcc()          = delete;

    rcc& operator=(const rcc&) = delete;
    rcc& operator=(rcc&&) = delete;
};
} // namespace stm32l4
} // namespace m4
} // namespace soc