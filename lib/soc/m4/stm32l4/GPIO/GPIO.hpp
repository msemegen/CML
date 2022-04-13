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
#include <soc/Peripheral.hpp>
#include <soc/m4/IRQ_config.hpp>
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

            friend Out;
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
        Out(GPIO* a_p_port)
            : p_port(a_p_port)
        {
        }

        GPIO* p_port;

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

            friend In;
        };

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

        GPIO* p_port;

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

            friend Analog;
        };

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

        GPIO* p_port;

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

        GPIO* p_port;

        friend GPIO;
    };

    class Interrupt : private cml::Non_copyable
    {
    private:
        template<std::uint8_t> struct H
        {
        };

    public:
        enum class Mode : std::uint32_t
        {
            interrupt,
            event
        };

        enum class Trigger_flag : std::uint32_t
        {
            rising  = 0x1,
            falling = 0x2,
        };

        struct Id
        {
            static H<0> _0;
            static H<1> _1;
            static H<2> _2;
            static H<3> _3;
            static H<4> _4;
            static H<5> _5_9;
            static H<6> _10_15;
        };

        struct Callback
        {
            using Function = void (*)(std::uint32_t a_pin, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        Interrupt(Interrupt&&) = default;
        Interrupt& operator=(Interrupt&&) = default;

        Interrupt()
            : idx(std::numeric_limits<decltype(this->idx)>::max())
            , irqn(static_cast<IRQn_Type>(std::numeric_limits<int32_t>::max()))
        {
        }

        Interrupt(H<0>)
            : idx(0)
            , irqn(EXTI0_IRQn)
        {
        }
        Interrupt(H<1>)
            : idx(1)
            , irqn(EXTI1_IRQn)
        {
        }
        Interrupt(H<2>)
            : idx(2)
            , irqn(EXTI2_IRQn)
        {
        }
        Interrupt(H<3>)
            : idx(3)
            , irqn(EXTI3_IRQn)
        {
        }
        Interrupt(H<4>)
            : idx(4)
            , irqn(EXTI4_IRQn)
        {
        }
        Interrupt(H<5>)
            : idx(5)
            , irqn(EXTI9_5_IRQn)
        {
        }
        Interrupt(H<6>)
            : idx(6)
            , irqn(EXTI15_10_IRQn)
        {
        }

        ~Interrupt()
        {
            if (0x0 != NVIC_GetEnableIRQ(this->irqn))
            {
                this->disable();
            }
        }

        void enable(const Callback& a_callback, const IRQ_config& a_irq_config);
        void disable();

        void attach(const GPIO& a_port, std::uint32_t a_pin, Trigger_flag a_trigger, Mode a_mode);
        void attach(const GPIO::In::Pin& a_pin, Trigger_flag a_trigger, Mode a_mode)
        {
            this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_mode);
        }
        void attach(const GPIO::Out::Pin& a_pin, Trigger_flag a_trigger, Mode a_mode)
        {
            this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_mode);
        }
        void attach(const GPIO::Alternate_function::Pin& a_pin, Trigger_flag a_trigger, Mode a_mode)
        {
            this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_mode);
        }

        void deattach(const GPIO& a_port, std::uint32_t a_pin);
        void deattach(const GPIO::In::Pin& a_pin)
        {
            this->deattach(*(a_pin.get_port()), a_pin.get_id());
        }
        void deattach(const GPIO::Out::Pin& a_pin)
        {
            this->deattach(*(a_pin.get_port()), a_pin.get_id());
        }
        void deattach(const GPIO::Alternate_function::Pin& a_pin)
        {
            this->deattach(*(a_pin.get_port()), a_pin.get_id());
        }

    private:
        std::uint32_t idx;
        IRQn_Type irqn;

        friend GPIO;
    };

    GPIO()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , flags(0u)
        , out(nullptr)
        , in(nullptr)
        , analog(nullptr)
        , alternate_function(nullptr)
    {
    }

    ~GPIO()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable()
    {
        cml::bit::set(&(this->flags), 31u);
    }

    void disable()
    {
        cml::bit::clear(&(this->flags), 31u);
    }

    bool is_pin_taken(std::uint8_t a_id) const
    {
        return cml::bit::is(this->flags, a_id);
    }

    bool is_enabled() const
    {
        return cml::bit::is(this->flags, 31u);
    }

    bool is_created()
    {
        return std::numeric_limits<decltype(this->idx)>::max() != this->idx && nullptr != this->p_registers;
    }

    explicit operator GPIO_TypeDef*()
    {
        return this->p_registers;
    }

private:
    GPIO(std::size_t a_idx, GPIO_TypeDef* a_p_registers)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , flags(0u)
        , out(this)
        , in(this)
        , analog(this)
        , alternate_function(this)
    {
    }

    void take_pin(std::uint8_t a_id)
    {
        cml::bit::set(&(this->flags), a_id);
    }

    void give_pin(std::uint8_t a_id)
    {
        cml::bit::clear(&(this->flags), a_id);
    }

    std::uint32_t idx;
    GPIO_TypeDef* p_registers;

    std::uint32_t flags;

    friend Out;
    friend In;
    friend Analog;
    friend Alternate_function;
    template<typename Periph_t, std::size_t id> friend class soc::Peripheral;

public:
    Out out;
    In in;
    Analog analog;
    Alternate_function alternate_function;
};

template<std::size_t id> class rcc<GPIO, id> : private cml::Non_constructible
{
public:
    static void enable(bool a_enable_in_lp) = delete;
    static void disable()                   = delete;
};

constexpr GPIO::Interrupt::Trigger_flag operator|(GPIO::Interrupt::Trigger_flag a_f1,
                                                  GPIO::Interrupt::Trigger_flag a_f2)
{
    return static_cast<GPIO::Interrupt::Trigger_flag>(static_cast<std::uint32_t>(a_f1) |
                                                      static_cast<std::uint32_t>(a_f2));
}
constexpr GPIO::Interrupt::Trigger_flag operator&(GPIO::Interrupt::Trigger_flag a_f1,
                                                  GPIO::Interrupt::Trigger_flag a_f2)
{
    return static_cast<GPIO::Interrupt::Trigger_flag>(static_cast<std::uint32_t>(a_f1) &
                                                      static_cast<std::uint32_t>(a_f2));
}
constexpr GPIO::Interrupt::Trigger_flag operator|=(GPIO::Interrupt::Trigger_flag& a_f1,
                                                   GPIO::Interrupt::Trigger_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace stm32l4
} // namespace m4
} // namespace soc