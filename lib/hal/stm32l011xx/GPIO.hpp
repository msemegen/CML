#pragma once

/*
    Name: GPIO.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l0xx.h>

//cml
#include <common/bit.hpp>
#include <common/integer.hpp>
#include <debug/assert.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

class GPIO
{
public:

    enum class Id : common::uint32
    {
        a = 0,
        b = 1,
        c = 2,
    };

    enum class Level : common::uint32
    {
        low  = 0x0u,
        high = 0x1u
    };

    enum class Mode : common::uint32
    {
        push_pull  = 0,
        open_drain = 1,
        unknown
    };

    enum class Pull : common::uint32
    {
        none    = 0x0u,
        up      = 0x1u,
        down    = 0x2u,
        unknown
    };

    enum class Speed : common::uint32
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

    GPIO()            = delete;
    GPIO(GPIO&&)      = default;
    GPIO(const GPIO&) = default;

    GPIO& operator = (GPIO&&)      = default;
    GPIO& operator = (const GPIO&) = default;

    void enable();
    void disable();

    bool is_enabled() const
    {
        return common::get_bit(this->flags, 31);
    }

    Id get_id() const
    {
        return this->id;
    }

    bool is_pin_taken(common::uint8 a_pin) const
    {
        return common::get_bit(this->flags, a_pin);
    }

    explicit operator GPIO_TypeDef*()
    {
        return this->p_gpio;
    }

private:

    common::uint32 to_index(Id a_id) const
    {
        return static_cast<common::uint32>(a_id);
    }

    void take_pin(common::uint8 a_pin)
    {
        common::set_bit(&(this->flags), a_pin);
    }

    void give_pin(common::uint8 a_pin)
    {
        common::clear_bit(&(this->flags), a_pin);
    }


private:

    Id id;

    common::uint32 flags;
    GPIO_TypeDef* p_gpio;

private:

    friend class Output_pin;
    friend class Input_pin;
    friend class Alternate_function_pin;
    friend class c_analog_pin;
};

class Output_pin
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

    Output_pin(GPIO* a_p_port, common::uint8 a_pin)
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

    Output_pin()                  = delete;
    Output_pin(Output_pin&&)      = default;
    Output_pin(const Output_pin&) = default;

    Output_pin& operator = (Output_pin&&)      = default;
    Output_pin& operator = (const Output_pin&) = default;

    void enable(const Config& a_config);
    void disable();

    void set_level(Level a_level);
    void toggle_level();

    void set_mode(Mode a_mode);
    void set_pull(Pull a_pull);
    void set_speed(Speed a_speed);

    Mode  get_mode() const;
    Pull  get_pull() const;
    Speed get_speed() const;

    GPIO* get_port() const
    {
        return this->p_port;
    }

    common::uint8 get_pin() const
    {
        return this->pin;
    }

private:

    GPIO* p_port;
    const common::uint8 pin;
};

class Input_pin
{
public:

    using Pull  = GPIO::Pull;
    using Level = GPIO::Level;

    struct Config
    {
        Pull pull = Pull::unknown;
    };

public:

    Input_pin(GPIO* a_p_port, common::uint8 a_pin)
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

    Input_pin()                 = delete;
    Input_pin(Input_pin&&)      = default;
    Input_pin(const Input_pin&) = default;

    Input_pin& operator = (Input_pin&&)      = default;
    Input_pin& operator = (const Input_pin&) = default;

    void enable(const Config& a_config);
    void disable();

    void set_pull(Pull a_pull);

    Pull  get_pull() const;
    Level get_level() const;

    GPIO* get_port() const
    {
        return this->p_port;
    }

    common::uint8 get_pin() const
    {
        return this->pin;
    }

private:

    GPIO* p_port;
    const common::uint8 pin;
};

class Alternate_function_pin
{
public:

    using Mode  = GPIO::Mode;
    using Pull  = GPIO::Pull;
    using Speed = GPIO::Speed;

    struct Config
    {
        Mode mode                = GPIO::Mode::unknown;
        Pull pull                = GPIO::Pull::unknown;
        Speed speed              = GPIO::Speed::unknown;
        common::uint32 function  = 0;
    };

public:

    Alternate_function_pin(GPIO* a_p_port, common::uint8 a_pin)
        : p_port(a_p_port)
        , pin(a_pin)
    {
        assert(nullptr != a_p_port);
        assert(a_pin < 16);
    }

    ~Alternate_function_pin()
    {
        this->disable();
    }

    Alternate_function_pin()                              = delete;
    Alternate_function_pin(Alternate_function_pin&&)      = default;
    Alternate_function_pin(const Alternate_function_pin&) = default;

    Alternate_function_pin& operator = (Alternate_function_pin&&)      = default;
    Alternate_function_pin& operator = (const Alternate_function_pin&) = default;

    void enable(const Config& a_config);
    void disable();

    void set_mode(Mode a_mode);
    void set_pull(Pull a_pull);
    void set_speed(Speed a_speed);
    void set_function(common::uint32 a_function);

    GPIO::Mode   get_mode() const;
    GPIO::Pull   get_pull() const;
    GPIO::Speed  get_speed() const;
    common::uint32 get_function() const;

private:

    GPIO* p_port;
    const common::uint8 pin;
};

} // namespace stm32l011xx
} // namespace hal
} // namespace cml