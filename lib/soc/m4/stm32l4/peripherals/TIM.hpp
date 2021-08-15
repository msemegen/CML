#pragma once

/*
 *   Name: TIM.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// soc
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

class TIM : private cml::Non_constructible
{
public:
    class Basic : private cml::Non_copyable
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
        Basic(Id a_id)
            : id(a_id)
        {
        }

        ~Basic()
        {
            this->stop();
            this->disable_time_base();
        }

        void enable_time_base(const Config& a_config, uint32_t a_irq_priority);
        void disable_time_base();

        void start();
        void stop();

        void register_overload_callback(const Overload_callback& a_callback);
        void unregister_overload_callback();

        bool is_overload_event() const;

    private:
        Id id;

        Overload_callback overload_callback;
        friend void interrupt_handler(Basic* a_p_this);
    };

    class General_puprose : private cml::Non_copyable
    {
    public:
        enum class Id
        {
            _2,
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
            _3,
#endif
            _15,
            _16
        };

    private:
        friend void interrupt_handler(General_puprose* a_p_this);
    };
};

} // namespace peripherals
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {

template<> class rcc<peripherals::TIM::Basic> : private cml::Non_constructible
{
public:
    static void enable(peripherals::TIM::Basic::Id a_id, bool a_enable_in_lp);
    static void disable(peripherals::TIM::Basic::Id a_id);
};

} // namespace stm32l4
} // namespace m4
} // namespace soc
