#pragma once

/*
 *   Name: Basic_timer.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// soc
#include <soc/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

class Basic_timer : private cml::Non_copyable
{
public:
    enum class Id
    {
        _6,
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
        _7
#endif
    };

    struct Overload_callback
    {
        using Function = void (*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Config
    {
        uint16_t prescaler   = 0u;
        uint16_t auto_reload = 0u;
    };

public:
    Basic_timer(Id a_id)
        : id(a_id)
    {
    }

    ~Basic_timer()
    {
        this->stop();
        this->disable();
    }

    void enable(const Config& a_config, uint32_t a_irq_priority);
    void disable();

    void start();
    void stop();

    void register_overload_callback(const Overload_callback& a_callback);
    void unregister_overload_callback();

    bool is_overload_event() const;

private:
    Id id;

    Overload_callback overload_callback;

private:
    friend void interrupt_handler(Basic_timer* a_p_this);
};

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc

namespace soc {
namespace stm32l4 {
template<> struct rcc<peripherals::Basic_timer>
{
    static void enable(peripherals::Basic_timer::Id a_id, bool a_enable_in_lp);
    static void disable(peripherals::Basic_timer::Id a_id);
};
} // namespace stm32l4
} // namespace soc
