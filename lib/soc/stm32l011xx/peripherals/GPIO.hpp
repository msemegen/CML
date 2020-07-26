#pragma once

/*
    Name: GPIO.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

//externals
#include <stm32l0xx.h>

//cml
#include <cml/bit.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/debug/assert.hpp>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

struct pin
{
    pin()            = delete;
    pin(pin&&)       = delete;
    pin(const pin&&) = delete;

    pin& operator = (pin&&)      = delete;
    pin& operator = (const pin&) = delete;

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
        none    = 0x0u,
        up      = 0x1u,
        down    = 0x2u,
        unknown
    };

    enum class Speed : uint32_t
    {
        low     = 0x0u,
        medium  = 0x1u,
        high    = 0x2u,
        ultra   = 0x3u,
        unknown
    };

    class In : private cml::Non_copyable
    {
    public:

        In(GPIO* a_p_port, uint8_t a_pin)
            : p_port(a_p_port)
            , pin(a_pin)
        {
            assert(nullptr != a_p_port);
            assert(a_pin < 16);
        }

        ~In()
        {
            this->disable();
        }

        void enable(Pull a_pull);
        void disable();

        void set_pull(Pull a_pull);

        Pull  get_pull() const;
        Level get_level() const;

        GPIO* get_port() const
        {
            return this->p_port;
        }

        uint8_t get_pin() const
        {
            return this->pin;
        }

    private:

        GPIO* p_port;
        const uint8_t pin;
    };

    class Out : private cml::Non_copyable
    {
    public:

        struct Config
        {
            Mode  mode = Mode::unknown;
            Pull  pull = Pull::unknown;
            Speed speed = Speed::unknown;
        };

    public:

        Out(GPIO* a_p_port, uint8_t a_pin)
            : p_port(a_p_port)
            , pin(a_pin)
        {
            assert(nullptr != a_p_port);
            assert(a_pin < 16);
        }

        ~Out()
        {
            this->disable();
        }

        void enable(const Config& a_config);
        void disable();

        void set_level(Level a_level);
        void toggle_level();

        void set_mode(Mode a_mode);
        void set_pull(Pull a_pull);
        void set_speed(Speed a_speed);

        Level get_level() const;
        Mode  get_mode()  const;
        Pull  get_pull()  const;
        Speed get_speed() const;

        GPIO* get_port() const
        {
            return this->p_port;
        }

        uint8_t get_pin() const
        {
            return this->pin;
        }

    private:

        GPIO* p_port;
        const uint8_t pin;
    };

    class Analog : private cml::Non_copyable
    {
    public:

        Analog(GPIO* a_p_port, uint8_t a_pin)
            : p_port(a_p_port)
            , pin(a_pin)
        {
            assert(nullptr != a_p_port);
            assert(a_pin < 16);
        }

        void enable(Pull a_pull);
        void disable();

        void set_pull(Pull a_pull);

        Pull get_pull() const;

        GPIO* get_port() const
        {
            return this->p_port;
        }

        uint8_t get_pin() const
        {
            return this->pin;
        }

    private:

        GPIO* p_port;
        const uint8_t pin;
    };

    class af_controller
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

        af_controller()                      = delete;
        af_controller(af_controller&&)       = delete;
        af_controller(const af_controller&&) = delete;

        af_controller& operator = (af_controller&&)      = delete;
        af_controller& operator = (const af_controller&) = delete;

        static void enable(GPIO* a_p_port, uint32_t a_pin, const Config& a_config);
        static void disable(GPIO* a_p_port, uint32_t a_pin);
    };
};

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
    {}

    ~GPIO()
    {
        this->disable();
    }

    void enable();
    void disable();

    bool is_enabled() const
    {
        return cml::is_bit(this->flags, 31);
    }

    Id get_id() const
    {
        return this->id;
    }

    bool is_pin_taken(uint8_t a_pin) const
    {
        return cml::is_bit(this->flags, a_pin);
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

    friend pin::In;
    friend pin::Out;
    friend pin::Analog;
    friend pin::af_controller;
};

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc