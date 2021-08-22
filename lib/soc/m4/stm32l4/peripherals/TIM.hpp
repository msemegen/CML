#pragma once

/*
 *   Name: TIM.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

class TIM : private cml::Non_constructible
{
public:
    struct IRQ
    {
        enum class Mode : uint32_t
        {
            disabled,
            enabled
        };

        Mode mode                 = cml::various::get_enum_incorrect_value<Mode>();
        uint32_t preempt_priority = 0;
        uint32_t sub_priority     = 0;
    };

    struct Overload_callback
    {
        using Function = void (*)(void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    class Basic : private cml::Non_constructible
    {
    public:
        enum class Id
        {
            _6,
#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
            _7
#endif
        };

        struct Time_base
        {
            enum class Mode : uint32_t
            {
                disabled,
                enabled
            };

            enum class Autoreload_preload : uint32_t
            {
                disabled = 0x0u,
                enabled  = TIM_CR1_ARPE,
                none
            };

            Mode mode                      = cml::various::get_enum_incorrect_value<Mode>();
            Autoreload_preload arr_preload = cml::various::get_enum_incorrect_value<Autoreload_preload>();
            uint16_t prescaler             = 0u;
        };

        class _6
        {
        public:
            _6() {}
            ~_6()
            {
                this->set_IRQ({ IRQ::Mode::disabled, 0 });
                this->unregister_overload_callback();
                this->stop();
                this->set_time_base({ Time_base::Mode::disabled, Time_base::Autoreload_preload::none, 0 });
            }

            void set_IRQ(const TIM::IRQ& a_config);
            void set_time_base(const Time_base& a_config);

            void start(uint16_t a_auto_reload);
            void stop();

            void register_overload_callback(const Overload_callback& a_callback);
            void unregister_overload_callback();

            bool is_overload_event() const;
            IRQ get_IRQ() const;

            Id get_Id() const
            {
                return Id::_6;
            }

        private:
            Overload_callback overload_callback;
            friend void basic_timer6_interrupt_handler(Basic::_6* a_p_this);
        };

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
        class _7
        {
        public:
            constexpr Id get_Id() const
            {
                return Id::_7;
            }
        };
#endif
    };

    class General_purpose : private cml::Non_constructible
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

        class _2 : private cml::Non_copyable
        {
        public:
            struct Time_base
            {
                enum class Mode : uint32_t
                {
                    disabled,
                    enabled
                };

                enum class Autoreload_preload : uint32_t
                {
                    disabled = 0x0u,
                    enabled  = TIM_CR1_ARPE,
                    none
                };

                enum class Direction : uint32_t
                {
                    up   = 0x0u,
                    down = TIM_CR1_DIR,
                    none
                };

                enum class Center_aligned_type : uint32_t
                {
                    none = 0x0u,
                    _1   = TIM_CR1_CMS_0,
                    _2   = TIM_CR1_CMS_1,
                    _3   = TIM_CR1_CMS_0 | TIM_CR1_CMS_1
                };

                Mode mode                          = cml::various::get_enum_incorrect_value<Mode>();
                Autoreload_preload arr_preload     = cml::various::get_enum_incorrect_value<Autoreload_preload>();
                Direction direction                = cml::various::get_enum_incorrect_value<Direction>();
                Center_aligned_type center_aligned = cml::various::get_enum_incorrect_value<Center_aligned_type>();
                uint16_t prescaler                 = 0u;
            };

        public:
            _2() {}
            ~_2()
            {
                this->set_IRQ({ IRQ::Mode::disabled, 0 });
                this->unregister_overload_callback();
                this->stop();
                this->set_time_base({ Time_base::Mode::disabled,
                                      Time_base::Autoreload_preload::none,
                                      Time_base::Direction::none,
                                      Time_base::Center_aligned_type::none,
                                      0 });
            }

            void set_IRQ(const IRQ& a_config);
            void set_time_base(const Time_base& a_config);

            void start(uint16_t a_auto_reload);
            void stop();

            void register_overload_callback(const Overload_callback& a_callback);
            void unregister_overload_callback();

            bool is_overload_event() const;
            IRQ get_IRQ() const;

            constexpr Id get_Id() const
            {
                return Id::_2;
            }

        private:
        };

#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        class _3 : private cml::Non_copyable
        {
        public:
            using Time_base = _2::Time_base;

            constexpr Id get_Id() const
            {
                return Id::_3;
            }
        };
#endif
        class _15 : private cml::Non_copyable
        {
        public:
            constexpr Id get_Id() const
            {
                return Id::_15;
            }
        };

        class _16 : private cml::Non_copyable
        {
        public:
            constexpr Id get_Id() const
            {
                return Id::_16;
            }
        };
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

    static void enable(const peripherals::TIM::Basic::_6& a_timer, bool a_enable_in_lp)
    {
        enable(a_timer.get_Id(), a_enable_in_lp);
    }

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
    static void enable(const peripherals::TIM::Basic::_7& a_timer, bool a_enable_in_lp)
    {
        enable(a_timer.get_Id(), a_enable_in_lp);
    }
#endif

    static void disable(const peripherals::TIM::Basic::_6& a_timer)
    {
        disable(a_timer.get_Id());
    }

#if defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)
    static void disable(const peripherals::TIM::Basic::_7& a_timer)
    {
        disable(a_timer.get_Id());
    }
#endif
};

template<> class rcc<peripherals::TIM::General_purpose> : private cml::Non_constructible
{
public:
    static void enable(peripherals::TIM::General_purpose::Id a_id, bool a_enable_in_lp);
    static void disable(peripherals::TIM::General_purpose::Id a_id);

    static void enable(peripherals::TIM::General_purpose::_2& a_timer, bool a_enable_in_lp)
    {
        enable(a_timer.get_Id(), a_enable_in_lp);
    }

#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    static void enable(peripherals::TIM::General_purpose::_3& a_timer, bool a_enable_in_lp)
    {
        enable(a_timer.get_Id(), a_enable_in_lp);
    }
#endif

    static void enable(peripherals::TIM::General_purpose::_15& a_timer, bool a_enable_in_lp)
    {
        enable(a_timer.get_Id(), a_enable_in_lp);
    }

    static void enable(peripherals::TIM::General_purpose::_16& a_timer, bool a_enable_in_lp)
    {
        enable(a_timer.get_Id(), a_enable_in_lp);
    }

    static void disable(peripherals::TIM::General_purpose::_2& a_timer)
    {
        disable(a_timer.get_Id());
    }

#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    static void disable(peripherals::TIM::General_purpose::_3& a_timer)
    {
        disable(a_timer.get_Id());
    }
#endif

    static void disable(peripherals::TIM::General_purpose::_15& a_timer)
    {
        disable(a_timer.get_Id());
    }

    static void disable(peripherals::TIM::General_purpose::_16& a_timer)
    {
        disable(a_timer.get_Id());
    }
};

} // namespace stm32l4
} // namespace m4
} // namespace soc
