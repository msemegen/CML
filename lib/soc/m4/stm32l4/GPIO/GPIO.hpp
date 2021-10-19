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
#include <soc/Handle.hpp>
#include <soc/m4/stm32l4/rcc.hpp>

// cml
#include <cml/Non_constructible.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>
#include <cml/various.hpp>

namespace soc {
namespace m4 {
namespace stm32l4 {
class GPIO : private cml::Non_copyable
{
public:
    enum class Level : std::uint32_t
    {
        low  = 0x0u,
        high = 0x1u
    };

    enum class Mode : std::uint32_t
    {
        push_pull  = 0x0u,
        open_drain = 0x1u,
    };

    enum class Pull : std::uint32_t
    {
        none = 0x0u,
        up   = 0x1u,
        down = 0x2u,
    };

    enum class Speed : std::uint32_t
    {
        low    = 0x0u,
        medium = 0x1u,
        high   = 0x2u,
        ultra  = 0x3u,
    };

    struct id : private cml::Non_constructible
    {
        constexpr static auto a = Handle<GPIOA_BASE> {};
        constexpr static auto b = Handle<GPIOB_BASE> {};
        constexpr static auto c = Handle<GPIOC_BASE> {};
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        constexpr static auto d = Handle<GPIOD_BASE> {};
#endif
#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
        constexpr static auto e = Handle<GPIOE_BASE> {};
#endif
        constexpr static auto h = Handle<GPIOH_BASE> {};
    };

    class Out : private cml::Non_copyable
    {
    public:
        struct Enable_config
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
                , id(0xFFu)
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

            std::uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint8_t id;

        private:
            friend Out;
        };

    public:
        void enable(std::uint32_t a_id, const Enable_config& a_enable_config, Pin* a_p_pin = nullptr);
        void disable(std::uint32_t a_id);

        void disable(Pin* p_pin)
        {
            this->disable(p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFFu;
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
                , id(0xFFu)
            {
            }

            void set_pull(Pull a_pull);

            Pull get_pull() const;
            Level get_level() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }

            std::uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint8_t id;

        private:
            friend In;
        };

    public:
        void enable(std::uint32_t a_id, Pull a_pull, Pin* a_p_pin = nullptr);
        void disable(std::uint32_t a_id);

        void disable(Pin* p_pin)
        {
            this->disable(p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFFu;
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
                , id(0xFFu)
            {
            }

            void set_pull(Pull a_pull);

            Pull get_pull() const;

            GPIO* get_port() const
            {
                return this->p_port;
            }

            std::uint8_t get_id() const
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
        void enable(std::uint32_t a_id, Pull a_pull, Pin* a_p_pin = nullptr);
        void disable(std::uint32_t a_id);

        void disable(Pin* p_pin)
        {
            this->disable(p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFFu;
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
        struct Enable_config
        {
            Mode mode   = cml::various::get_enum_incorrect_value<Mode>();
            Pull pull   = cml::various::get_enum_incorrect_value<Pull>();
            Speed speed = cml::various::get_enum_incorrect_value<Speed>();

            std::uint32_t function = 0;
        };

        class Pin
        {
        public:
            Pin()
                : p_port(nullptr)
                , id(0xFFu)
            {
            }

            void set_mode(Mode a_mode);
            void set_pull(Pull a_pull);
            void set_speed(Speed a_speed);
            void set_function(std::uint32_t a_function);

            Mode get_mode() const;
            Pull get_pull() const;
            Speed get_speed() const;

            std::uint32_t get_function() const
            {
                return this->function;
            }

            GPIO* get_port() const
            {
                return this->p_port;
            }

            std::uint8_t get_id() const
            {
                return this->id;
            }

        private:
            GPIO* p_port;
            std::uint8_t id;

            std::uint32_t function;

        private:
            friend Alternate_function;
        };

        void enable(std::uint32_t a_id, const Enable_config& a_enable_config, Pin* a_p_pin = nullptr);
        void disable(std::uint32_t a_id);

        void disable(Pin* p_pin)
        {
            this->disable(p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFFu;
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

public:
    GPIO(Handle<GPIOA_BASE>)
        : idx(0u)
        , flags(0u)
        , p_registers(GPIOA)
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

    GPIO(Handle<GPIOB_BASE>)
        : idx(1u)
        , flags(0u)
        , p_registers(GPIOB)
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

    GPIO(Handle<GPIOC_BASE>)
        : idx(2u)
        , flags(0u)
        , p_registers(GPIOC)
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

    GPIO(Handle<GPIOH_BASE>)
        : idx(7)
        , flags(0)
        , p_registers(GPIOH)
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

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    GPIO(Handle<GPIOD_BASE>)
        : idx(3u)
        , flags(0u)
        , p_registers(GPIOD)
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
#endif
#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
    GPIO(Handle<GPIOE_BASE>)
        : idx(4u)
        , flags(0u)
        , p_registers(GPIOE)
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
#endif

    ~GPIO()
    {
        this->disable();
    }

    void enable()
    {
        cml::bit::set(&(this->flags), 31u);
    }

    void disable()
    {
        cml::bit::clear(&(this->flags), 31u);
    }

    std::uint32_t get_id() const
    {
        return this->idx;
    }

    bool is_pin_taken(std::uint8_t a_id) const
    {
        return cml::bit::is(this->flags, a_id);
    }

    bool is_enabled() const
    {
        return cml::bit::is(this->flags, 31u);
    }

    explicit operator GPIO_TypeDef*()
    {
        return this->p_registers;
    }

private:
    void take_pin(std::uint8_t a_id)
    {
        cml::bit::set(&(this->flags), a_id);
    }

    void give_pin(std::uint8_t a_id)
    {
        cml::bit::clear(&(this->flags), a_id);
    }

private:
    std::uint32_t idx;

    std::uint32_t flags;
    GPIO_TypeDef* p_registers;

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

template<> class rcc<GPIO> : private cml::Non_constructible
{
public:
    template<std::uint32_t peripheral_base_address>
    static void enable(Handle<peripheral_base_address>, bool a_enable_in_lp);
    template<std::uint32_t peripheral_base_address> static void disable(Handle<peripheral_base_address>);
};

template<> void rcc<GPIO>::enable<GPIOA_BASE>(Handle<GPIOA_BASE>, bool a_enable_in_lp);
template<> void rcc<GPIO>::enable<GPIOB_BASE>(Handle<GPIOB_BASE>, bool a_enable_in_lp);
template<> void rcc<GPIO>::enable<GPIOC_BASE>(Handle<GPIOC_BASE>, bool a_enable_in_lp);
template<> void rcc<GPIO>::enable<GPIOH_BASE>(Handle<GPIOH_BASE>, bool a_enable_in_lp);
template<> void rcc<GPIO>::disable<GPIOA_BASE>(Handle<GPIOA_BASE>);
template<> void rcc<GPIO>::disable<GPIOB_BASE>(Handle<GPIOB_BASE>);
template<> void rcc<GPIO>::disable<GPIOC_BASE>(Handle<GPIOC_BASE>);
template<> void rcc<GPIO>::disable<GPIOH_BASE>(Handle<GPIOH_BASE>);

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<GPIO>::enable<GPIOD_BASE>(Handle<GPIOD_BASE>, bool a_enable_in_lp);
template<> void rcc<GPIO>::disable<GPIOD_BASE>(Handle<GPIOD_BASE>);
#endif
#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
template<> void rcc<GPIO>::enable<GPIOE_BASE>(Handle<GPIOE_BASE>, bool a_enable_in_lp);
template<> void rcc<GPIO>::disable<GPIOE_BASE>(Handle<GPIOE_BASE>);
#endif
} // namespace stm32l4
} // namespace m4
} // namespace soc