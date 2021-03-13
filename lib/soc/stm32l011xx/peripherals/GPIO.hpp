#pragma once

/*
    Name: GPIO.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// externals
#include <stm32l0xx.h>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/bit.hpp>
#include <cml/debug/assertion.hpp>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

class GPIO : private cml::Non_copyable
{
public:
    enum class Id : uint32_t
    {
        a = 0,
        b = 1,
        c = 2,
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
        unknown
    };

    enum class Pull : uint32_t
    {
        none = 0x0u,
        up   = 0x1u,
        down = 0x2u,
        unknown
    };

    enum class Speed : uint32_t
    {
        low    = 0x0u,
        medium = 0x1u,
        high   = 0x2u,
        ultra  = 0x3u,
        unknown
    };

    class Out : private cml::Non_copyable
    {
    public:
        struct Config
        {
            Mode mode   = Mode::unknown;
            Pull pull   = Pull::unknown;
            Speed speed = Speed::unknown;
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
            enum class Interrupt_mode_flag : uint32_t
            {
                rising  = 0x1,
                falling = 0x2,
            };

            struct Interrupt_callback
            {
                using Function = void (*)(Level a_level, void* a_p_user_data);

                Function function = nullptr;
                void* p_user_data = nullptr;

                void operator()(Level a_level)
                {
                    cml_assert(nullptr != this->function);

                    this->function(a_level, this->p_user_data);
                }
            };

        public:
            Pin()
                : p_port(nullptr)
                , id(0xFF)
            {
            }

            void set_pull(Pull a_pull);

            Pull get_pull() const;
            Level get_level() const;

            void register_interrupt_callback(Interrupt_mode_flag a_mode, const Interrupt_callback& a_callback);
            void unregister_interrupt_callback();

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
            Mode mode   = Mode::unknown;
            Pull pull   = Pull::unknown;
            Speed speed = Speed::unknown;

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
        return cml::bit::is(this->flags, 31);
    }

    Id get_id() const
    {
        return this->id;
    }

    bool is_pin_taken(uint8_t a_pin) const
    {
        return cml::bit::is(this->flags, a_pin);
    }

    explicit operator GPIO_TypeDef*()
    {
        return this->p_gpio;
    }

private:
    void take_pin(uint8_t a_pin)
    {
        cml::bit::set(&(this->flags), a_pin);
    }

    void give_pin(uint8_t a_pin)
    {
        cml::bit::clear(&(this->flags), a_pin);
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

constexpr GPIO::In::Pin::Interrupt_mode_flag operator|(GPIO::In::Pin::Interrupt_mode_flag a_f1,
                                                       GPIO::In::Pin::Interrupt_mode_flag a_f2)
{
    return static_cast<GPIO::In::Pin::Interrupt_mode_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr GPIO::In::Pin::Interrupt_mode_flag operator&(GPIO::In::Pin::Interrupt_mode_flag a_f1,
                                                       GPIO::In::Pin::Interrupt_mode_flag a_f2)
{
    return static_cast<GPIO::In::Pin::Interrupt_mode_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr GPIO::In::Pin::Interrupt_mode_flag operator|=(GPIO::In::Pin::Interrupt_mode_flag& a_f1,
                                                        GPIO::In::Pin::Interrupt_mode_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc