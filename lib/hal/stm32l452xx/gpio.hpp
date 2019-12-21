#pragma once

/*
    Name: gpio.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l4xx.h>

//cml
#include <common/assert.hpp>
#include <common/bit.hpp>
#include <common/integer.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class c_gpio
{
public:

    enum class e_periph : common::uint32
    {
        a = 0,
        b = 1,
        c = 2,
        d = 3,
        e = 4,
        h = 5
    };

    enum class e_level : common::uint32
    {
        low  = 0x0u,
        high = 0x1u
    };

    enum class e_mode : common::uint32
    {
        push_pull  = 0,
        open_drain = 1,
        unknown
    };

    enum class e_pull : common::uint32
    {
        none    = 0x0u,
        up      = 0x1u,
        down    = 0x2u,
        unknown
    };

    enum class e_speed : common::uint32
    {
        low     = 0x0u,
        medium  = 0x1u,
        high    = 0x2u,
        ultra   = 0x3u,
        unknown
    };

public:

    c_gpio(e_periph a_periph)
        : periph(a_periph)
        , flags(0)
        , p_gpio(nullptr)
    {}

    ~c_gpio()
    {
        this->disable();
    }

    c_gpio()              = delete;
    c_gpio(c_gpio&&)      = default;
    c_gpio(const c_gpio&) = default;

    c_gpio& operator = (c_gpio&&)      = default;
    c_gpio& operator = (const c_gpio&) = default;

    void enable();
    void disable();

    bool is_enabled() const
    {
        return common::get_bit(this->flags, 31);
    }

    e_periph get_periph() const
    {
        return this->periph;
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

    common::uint32 to_index(e_periph a_periph) const
    {
        return static_cast<common::uint32>(a_periph);
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

    e_periph periph;

    common::uint32 flags;
    GPIO_TypeDef* p_gpio;

private:

    friend class c_output_pin;
    friend class c_input_pin;
    friend class c_alternate_function_pin;
    friend class c_analog_pin;
};

class c_output_pin
{
public:

    struct s_config
    {
        c_gpio::e_mode  mode  = c_gpio::e_mode::unknown;
        c_gpio::e_pull  pull  = c_gpio::e_pull::unknown;
        c_gpio::e_speed speed = c_gpio::e_speed::unknown;
    };

public:

    c_output_pin(c_gpio* a_p_port, common::uint8 a_pin)
        : p_port(a_p_port)
        , pin(a_pin)
    {
        _assert(nullptr != a_p_port);
        _assert(a_pin < 16);
    }

    ~c_output_pin()
    {
        this->disable();
    }

    c_output_pin()                    = delete;
    c_output_pin(c_output_pin&&)      = default;
    c_output_pin(const c_output_pin&) = default;

    c_output_pin& operator = (c_output_pin&&)      = default;
    c_output_pin& operator = (const c_output_pin&) = default;

    void enable(const s_config& a_config);
    void disable();

    void set_level(c_gpio::e_level a_level);
    void toggle_level();

    void set_mode(c_gpio::e_mode a_mode);
    void set_pull(c_gpio::e_pull a_pull);
    void set_speed(c_gpio::e_speed a_speed);

    c_gpio::e_mode  get_mode() const;
    c_gpio::e_pull  get_pull() const;
    c_gpio::e_speed get_speed() const;

    c_gpio* get_port() const
    {
        return this->p_port;
    }

    common::uint8 get_pin() const
    {
        return this->pin;
    }

private:

    c_gpio* p_port;
    const common::uint8 pin;
};

class c_input_pin
{
public:

    struct s_config
    {
        c_gpio::e_pull pull = c_gpio::e_pull::unknown;
    };

public:

    c_input_pin(c_gpio* a_p_port, common::uint8 a_pin)
        : p_port(a_p_port)
        , pin(a_pin)
    {
        _assert(nullptr != a_p_port);
        _assert(a_pin < 16);
    }

    ~c_input_pin()
    {
        this->disable();
    }

    c_input_pin()                   = delete;
    c_input_pin(c_input_pin&&)      = default;
    c_input_pin(const c_input_pin&) = default;

    c_input_pin& operator = (c_input_pin&&)      = default;
    c_input_pin& operator = (const c_input_pin&) = default;

    void enable(const s_config& a_config);
    void disable();

    c_gpio::e_level read_level() const;

    void set_pull(c_gpio::e_pull a_pull);

    c_gpio::e_pull get_pull() const;

    c_gpio* get_port() const
    {
        return this->p_port;
    }

    common::uint8 get_pin() const
    {
        return this->pin;
    }

private:

    c_gpio* p_port;
    const common::uint8 pin;
};

class c_alternate_function_pin
{
public:

    struct s_config
    {
        c_gpio::e_mode mode      = c_gpio::e_mode::unknown;
        c_gpio::e_pull pull      = c_gpio::e_pull::unknown;
        c_gpio::e_speed speed    = c_gpio::e_speed::unknown;
        common::uint32 function  = 0;
    };

public:

    c_alternate_function_pin(c_gpio* a_p_port, common::uint8 a_pin)
        : p_port(a_p_port)
        , pin(a_pin)
    {
        _assert(nullptr != a_p_port);
        _assert(a_pin < 16);
    }

    ~c_alternate_function_pin()
    {
        this->disable();
    }

    c_alternate_function_pin()                                = delete;
    c_alternate_function_pin(c_alternate_function_pin&&)      = default;
    c_alternate_function_pin(const c_alternate_function_pin&) = default;

    c_alternate_function_pin& operator = (c_alternate_function_pin&&)      = default;
    c_alternate_function_pin& operator = (const c_alternate_function_pin&) = default;

    void enable(const s_config& a_config);
    void disable();

    void set_mode(c_gpio::e_mode a_mode);
    void set_pull(c_gpio::e_pull a_pull);
    void set_speed(c_gpio::e_speed a_speed);
    void set_function(common::uint32 a_function);

    c_gpio::e_mode   get_mode() const;
    c_gpio::e_pull   get_pull() const;
    c_gpio::e_speed  get_speed() const;
    common::uint32 get_function() const;

private:

    c_gpio* p_port;
    const common::uint8 pin;
};

class c_analog_pin
{

};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml