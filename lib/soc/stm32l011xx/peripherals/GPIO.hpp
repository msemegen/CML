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
#include <cml/debug/assert.hpp>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

class GPIO;

struct pin
{
    pin()            = delete;
    pin(pin&&)       = delete;
    pin(const pin&&) = delete;

    pin& operator=(pin&&) = delete;
    pin& operator=(const pin&) = delete;

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

    class in;
    class out;
    class analog;
    class af;

    class In : private cml::Non_copyable
    {
    public:
        In()
            : p_port(nullptr)
            , id(0xFF)
        {
        }
        ~In() = default;

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
        friend class pin::in;
    };

    class Out : private cml::Non_copyable
    {
    public:
        Out()
            : p_port(nullptr)
            , id(0xFF)
        {
        }
        ~Out() = default;

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
        friend pin::out;
    };

    class Analog
    {
    public:
        Analog()
            : p_port(nullptr)
            , id(0xFF)
        {
        }

        ~Analog() = default;

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
        friend pin::analog;
    };

    class Af
    {
    public:
        Af()
            : p_port(nullptr)
            , id(0xFF)
        {
        }

        ~Af() = default;

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
        friend af;
    };

    class in
    {
    public:
        enum class Interrupt_mode : uint32_t
        {
            rising  = 0x1,
            falling = 0x2,
        };

        struct Interrupt_callback
        {
            using Function = void (*)(pin::Level a_level, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

    public:
        in()           = delete;
        in(in&&)       = delete;
        in(const in&&) = delete;

        in& operator=(in&&) = delete;
        in& operator=(const in&) = delete;

        static void enable(GPIO* a_p_port, uint32_t a_id, Pull a_pull, In* a_p_out_pin = nullptr);
        static void disable(GPIO* a_p_port, uint32_t a_id);

        static void disable(In* p_pin)
        {
            disable(p_pin->get_port(), p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFF;
        }


        static void enable_interrupt(GPIO* a_p_port,
                                     uint32_t a_id,
                                     Pull a_pull,
                                     Interrupt_mode a_mode,
                                     const Interrupt_callback& a_callback);

        static void disable_interrupt(const In& a_pin);
    };

    class out
    {
    public:
        struct Config
        {
            Mode mode   = Mode::unknown;
            Pull pull   = Pull::unknown;
            Speed speed = Speed::unknown;
        };

    public:
        out()            = delete;
        out(out&&)       = delete;
        out(const out&&) = delete;

        out& operator=(out&&) = delete;
        out& operator=(const out&) = delete;

        static void enable(GPIO* a_p_port, uint32_t a_id, const Config& a_config, Out* a_p_out_pin = nullptr);
        static void disable(GPIO* a_p_port, uint32_t a_id);

        static void disable(Out* p_pin)
        {
            disable(p_pin->get_port(), p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFF;
        }
    };

    class analog
    {
    public:
        analog()               = delete;
        analog(analog&&)       = delete;
        analog(const analog&&) = delete;

        analog& operator=(analog&&) = delete;
        analog& operator=(const analog&) = delete;

        static void enable(GPIO* a_p_port, uint32_t a_id, Pull a_pull, Analog* a_p_out_pin = nullptr);
        static void disable(GPIO* a_p_port, uint32_t a_id);

        static void disable(Analog* p_pin)
        {
            disable(p_pin->get_port(), p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFF;
        }
    };

    class af
    {
    public:
        struct Config
        {
            Mode mode   = Mode::unknown;
            Pull pull   = Pull::unknown;
            Speed speed = Speed::unknown;

            uint32_t function = 0;
        };

    public:
        af()           = delete;
        af(af&&)       = delete;
        af(const af&&) = delete;

        af& operator=(af&&) = delete;
        af& operator=(const af&) = delete;

        static void enable(GPIO* a_p_port, uint32_t a_id, const Config& a_config, Af* a_p_out_pin = nullptr);
        static void disable(GPIO* a_p_port, uint32_t a_id);

        static void disable(Af* p_pin)
        {
            disable(p_pin->get_port(), p_pin->get_id());

            p_pin->p_port = nullptr;
            p_pin->id     = 0xFF;
        }
    };
};

constexpr pin::in::Interrupt_mode operator|(pin::in::Interrupt_mode a_f1, pin::in::Interrupt_mode a_f2)
{
    return static_cast<pin::in::Interrupt_mode>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr pin::in::Interrupt_mode operator&(pin::in::Interrupt_mode a_f1, pin::in::Interrupt_mode a_f2)
{
    return static_cast<pin::in::Interrupt_mode>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr pin::in::Interrupt_mode operator|=(pin::in::Interrupt_mode& a_f1, pin::in::Interrupt_mode a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

class GPIO : private cml::Non_copyable
{
public:
    enum class Id : uint32_t
    {
        a = 0,
        b = 1,
        c = 2,
    };

public:
    GPIO(Id a_id)
        : id(a_id)
        , flags(0)
        , p_gpio(nullptr)
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
        return cml::is_bit_on(this->flags, 31);
    }

    Id get_id() const
    {
        return this->id;
    }

    bool is_pin_taken(uint8_t a_pin) const
    {
        return cml::is_bit_on(this->flags, a_pin);
    }

    explicit operator GPIO_TypeDef*()
    {
        return this->p_gpio;
    }

private:
    void take_pin(uint8_t a_pin)
    {
        cml::set_bit(&(this->flags), a_pin);
    }

    void give_pin(uint8_t a_pin)
    {
        cml::clear_bit(&(this->flags), a_pin);
    }

private:
    Id id;

    uint32_t flags;
    GPIO_TypeDef* p_gpio;

private:
    friend pin::in;
    friend pin::out;
    friend pin::analog;
    friend pin::af;
};

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc