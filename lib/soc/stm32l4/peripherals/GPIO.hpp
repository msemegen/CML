#pragma once

/*
 *   Name: GPIO.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// externals
#include <stm32l4xx.h>

// soc
#include <soc/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/various.hpp>

namespace soc {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

class GPIO : private cml::Non_copyable
{
public:
    enum class Id : uint32_t
    {
        a = 0,
        b = 1,
        c = 2,
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        d = 3,
#endif
#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
        e = 4,
#endif
        h = 7
    };

    enum class Level : uint32_t
    {
        low  = 0x0u,
        high = 0x1u
    };

    enum class Mode : uint32_t
    {
        push_pull  = 0,
        open_drain = 1,
    };

    enum class Pull : uint32_t
    {
        none = 0x0u,
        up   = 0x1u,
        down = 0x2u,
    };

    enum class Speed : uint32_t
    {
        low    = 0x0u,
        medium = 0x1u,
        high   = 0x2u,
        ultra  = 0x3u,
    };

    class Out : private cml::Non_copyable
    {
    public:
        struct Config
        {
            Mode mode   = cml::various::get_enum_incorrect_value<Mode>();
            Pull pull   = cml::various::get_enum_incorrect_value<Pull>();
            Speed speed = cml::various::get_enum_incorrect_value<Speed>();
        };

        class Pin : private cml::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFF)
            {
            }

            void set_level(Level a_level);
            void toggle_level();

            void set_mode(Mode a_mode);
            void set_pull(Pull a_pull);
            void set_speed(Speed a_speed);

            Level get_level() const;
            Mode get_mode() const;
            Pull get_pull() const;
            Speed get_speed() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }

            uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            uint8_t id;

        private:
            friend Out;
        };

    public:
        void enable(uint32_t a_id, const Config& a_config, Pin* a_p_pin = nullptr);
        void disable(uint32_t a_id);

        void disable(Pin* p_pin)
        {
            this->disable(p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFF;
        }

    private:
        Out(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

    private:
        GPIO* p_port;

    private:
        friend GPIO;
    };

    class In : private cml::Non_copyable
    {
    public:
        class Pin : private cml::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFF)
            {
            }

            void set_pull(Pull a_pull);

            Pull get_pull() const;
            Level get_level() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }

            uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            uint8_t id;

        private:
            friend In;
        };

    public:
        void enable(uint32_t a_id, Pull a_pull, Pin* a_p_pin = nullptr);
        void disable(uint32_t a_id);

        void disable(Pin* p_pin)
        {
            this->disable(p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFF;
        }

    private:
        In(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

    private:
        GPIO* p_port;

    private:
        friend GPIO;
    };

    class Analog : private cml::Non_copyable
    {
    public:
        class Pin : private cml::Non_copyable
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFF)
            {
            }

            void set_pull(Pull a_pull);

            Pull get_pull() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }

            uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            uint8_t id;

        private:
            friend Analog;
        };

    public:
        void enable(uint32_t a_id, Pull a_pull, Pin* a_p_pin = nullptr);
        void disable(uint32_t a_id);

        void disable(Pin* p_pin)
        {
            this->disable(p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFF;
        }

    private:
        Analog(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

    private:
        GPIO* p_port;

    private:
        friend GPIO;
    };

    class Alternate_function : cml::Non_copyable
    {
    public:
        struct Config
        {
            Mode mode   = cml::various::get_enum_incorrect_value<Mode>();
            Pull pull   = cml::various::get_enum_incorrect_value<Pull>();
            Speed speed = cml::various::get_enum_incorrect_value<Speed>();

            uint32_t function = 0;
        };

        class Pin
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFF)
            {
            }

            void set_mode(Mode a_mode);
            void set_pull(Pull a_pull);
            void set_speed(Speed a_speed);
            void set_function(uint32_t a_function);

            Mode get_mode() const;
            Pull get_pull() const;
            Speed get_speed() const;

            uint32_t get_function() const
            {
                return this->function;
            }

            GPIO* get_port() const
            {
                return this->p_port;
            }

            uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            uint8_t id;

            uint32_t function;

        private:
            friend Alternate_function;
        };

        void enable(uint32_t a_id, const Config& a_config, Pin* a_p_pin = nullptr);
        void disable(uint32_t a_id);

        void disable(Pin* p_pin)
        {
            this->disable(p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFF;
        }

    private:
        Alternate_function(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

    private:
        GPIO* p_port;

    private:
        friend GPIO;
    };

#ifdef EXTI
#undef EXTI
#endif

    class EXTI : private cml::Non_copyable
    {
    public:
        enum class Id
        {
            _0     = EXTI0_IRQn,
            _1     = EXTI1_IRQn,
            _2     = EXTI2_IRQn,
            _3     = EXTI3_IRQn,
            _4     = EXTI4_IRQn,
            _5_9   = EXTI9_5_IRQn,
            _10_15 = EXTI15_10_IRQn
        };

        enum class Trigger_flag : uint32_t
        {
            rising  = 0x1,
            falling = 0x2,
        };

        struct Callback
        {
            using Function = void (*)(uint32_t a_pin, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

    public:
        EXTI(Id a_id)
            : id(a_id)
        {
        }

        ~EXTI()
        {
            this->disable();
        }

        void enable(const Callback& a_callback, uint32_t a_priority);
        void disable();

        void attach(const GPIO& a_port, uint32_t a_pin, Trigger_flag a_trigger);
        void deattach(const GPIO& a_port, uint32_t a_pin);

    private:
        Id id;

        friend GPIO;
    };

public:
    GPIO(Id a_id)
        : id(a_id)
        , flags(0)
        , p_gpio(nullptr)
        , out(this)
        , in(this)
        , analog(this)
        , alternate_function(this)
        , p_out(&(this->out))
        , p_in(&(this->in))
        , p_analog(&(this->analog))
        , p_alternate_function(&(this->alternate_function))
    {
    }

    ~GPIO()
    {
        this->disable();
    }

    void enable();
    void disable();

    bool is_enabled() const
    {
        return cml::bit::is(this->flags, 31u);
    }

    Id get_id() const
    {
        return this->id;
    }

    bool is_pin_taken(uint8_t a_id) const
    {
        return cml::bit::is(this->flags, a_id);
    }

    explicit operator GPIO_TypeDef*()
    {
        return this->p_gpio;
    }

private:
    void take_pin(uint8_t a_id)
    {
        cml::bit::set(&(this->flags), a_id);
    }

    void give_pin(uint8_t a_id)
    {
        cml::bit::clear(&(this->flags), a_id);
    }

private:
    Id id;

    uint32_t flags;
    GPIO_TypeDef* p_gpio;

    Out out;
    In in;
    Analog analog;
    Alternate_function alternate_function;

private:
    friend Out;
    friend In;
    friend Analog;
    friend Alternate_function;

public:
    Out* const p_out;
    In* const p_in;
    Analog* const p_analog;
    Alternate_function* const p_alternate_function;
};

constexpr GPIO::EXTI::Trigger_flag operator|(GPIO::EXTI::Trigger_flag a_f1, GPIO::EXTI::Trigger_flag a_f2)
{
    return static_cast<GPIO::EXTI::Trigger_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}
constexpr GPIO::EXTI::Trigger_flag operator&(GPIO::EXTI::Trigger_flag a_f1, GPIO::EXTI::Trigger_flag a_f2)
{
    return static_cast<GPIO::EXTI::Trigger_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}
constexpr GPIO::EXTI::Trigger_flag operator|=(GPIO::EXTI::Trigger_flag& a_f1, GPIO::EXTI::Trigger_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc

namespace soc {
namespace stm32l4 {
template<> struct rcc<peripherals::GPIO>
{
    static void enable(peripherals::GPIO::Id a_id, bool a_enable_in_lp);
    static void disable(peripherals::GPIO::Id a_id);
};
} // namespace stm32l4
} // namespace soc