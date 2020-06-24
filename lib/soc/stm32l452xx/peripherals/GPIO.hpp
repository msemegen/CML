#pragma once

/*
    Name: GPIO.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l4xx.h>

//cml
#include <cml/bit.hpp>
#include <cml/integer.hpp>
#include <cml/Non_copyable.hpp>
#include <cml/debug/assert.hpp>

namespace soc {
namespace stm32l452xx {
namespace peripherals {

class GPIO : private cml::Non_copyable
{
public:

    enum class Id : cml::uint32
    {
        a = 0,
        b = 1,
        c = 2,
        d = 3,
        e = 4,
        h = 7
    };

    enum class Level : cml::uint32
    {
        low  = 0x0u,
        high = 0x1u
    };

    enum class Mode : cml::uint32
    {
        push_pull  = 0,
        open_drain = 1,
        unknown
    };

    enum class Pull : cml::uint32
    {
        none    = 0x0u,
        up      = 0x1u,
        down    = 0x2u,
        unknown
    };

    enum class Speed : cml::uint32
    {
        low     = 0x0u,
        medium  = 0x1u,
        high    = 0x2u,
        ultra   = 0x3u,
        unknown
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

    bool is_pin_taken(cml::uint8 a_pin) const
    {
        return cml::is_bit(this->flags, a_pin);
    }

    explicit operator GPIO_TypeDef*()
    {
        return this->p_gpio;
    }

private:

    void take_pin(cml::uint8 a_pin)
    {
        cml::set_bit(&(this->flags), a_pin);
    }

    void give_pin(cml::uint8 a_pin)
    {
        cml::clear_bit(&(this->flags), a_pin);
    }

private:

    Id id;

    cml::uint32 flags;
    GPIO_TypeDef* p_gpio;

private:

    friend class Output_pin;
    friend class Input_pin;
    friend class Alternate_function_pin;
    friend class Analog_pin;
};

class Output_pin : private cml::Non_copyable
{
public:

    using Mode  = GPIO::Mode;
    using Pull  = GPIO::Pull;
    using Speed = GPIO::Speed;
    using Level = GPIO::Level;

    struct Config
    {
        Mode  mode  = Mode::unknown;
        Pull  pull  = Pull::unknown;
        Speed speed = Speed::unknown;
    };

public:

    Output_pin(GPIO* a_p_port, cml::uint8 a_pin)
        : p_port(a_p_port)
        , pin(a_pin)
    {
        assert(nullptr != a_p_port);
        assert(a_pin < 16);
    }

    ~Output_pin()
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

    cml::uint8 get_pin() const
    {
        return this->pin;
    }

private:

    GPIO* p_port;
    const cml::uint8 pin;
};

class Input_pin : private cml::Non_copyable
{
public:

    using Pull = GPIO::Pull;
    using Level = GPIO::Level;

public:

    Input_pin(GPIO* a_p_port, cml::uint8 a_pin)
        : p_port(a_p_port)
        , pin(a_pin)
    {
        assert(nullptr != a_p_port);
        assert(a_pin < 16);
    }

    ~Input_pin()
    {
        this->disable();
    }

    void enable(Pull a_pull);
    void disable();

    void set_pull(Pull a_pull);

    Pull get_pull() const;
    Level get_level() const;

    GPIO* get_port() const
    {
        return this->p_port;
    }

    cml::uint8 get_pin() const
    {
        return this->pin;
    }

private:

    GPIO* p_port;
    const cml::uint8 pin;
};

class Alternate_function_pin : private cml::Non_copyable
{
public:

    using Mode  = GPIO::Mode;
    using Pull  = GPIO::Pull;
    using Speed = GPIO::Speed;

    struct Config
    {
        Mode mode             = Mode::unknown;
        Pull pull             = Pull::unknown;
        Speed speed           = Speed::unknown;
        cml::uint32 function  = 0;
    };

public:

    Alternate_function_pin(GPIO* a_p_port, cml::uint8 a_pin)
        : p_port(a_p_port)
        , pin(a_pin)
        , function(0)
    {
        assert(nullptr != a_p_port);
        assert(a_pin < 16);
    }

    ~Alternate_function_pin()
    {
        this->disable();
    }

    void enable(const Config& a_config);
    void disable();

    void set_mode(Mode a_mode);
    void set_pull(Pull a_pull);
    void set_speed(Speed a_speed);
    void set_function(cml::uint32 a_function);

    Mode  get_mode()  const;
    Pull  get_pull()  const;
    Speed get_speed() const;

    cml::uint32 get_function() const
    {
        return this->function;
    }

private:

    GPIO* p_port;
    const cml::uint8 pin;

    cml::uint32 function;
};

class Analog_pin : private cml::Non_copyable
{
public:

    using Pull = GPIO::Pull;

public:

    Analog_pin(GPIO* a_p_port, cml::uint8 a_pin)
        : p_port(a_p_port)
        , pin(a_pin)
    {
        assert(nullptr != a_p_port);
        assert(a_pin < 16);
    }

    ~Analog_pin()
    {
        this->disable();
    }

    void enable(Pull a_pull);
    void disable();

    void set_pull(Pull a_pull);

    Pull get_pull() const;

    GPIO* get_port() const
    {
        return this->p_port;
    }

    cml::uint8 get_pin() const
    {
        return this->pin;
    }

private:

    GPIO* p_port;
    const cml::uint8 pin;
};

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc