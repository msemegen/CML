#pragma once

/*
    Name: systick.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>
#include <common/time_tick.hpp>

namespace cml {
namespace hal {

class Systick
{
public:

    void enable(common::uint32 a_priority);
    void disable();

    bool is_enabled() const;

    void reset_counter()
    {
        this->cnt = 0;
    }

    common::time_tick get_counter()
    {
        return this->cnt;
    }

    static Systick& get_instance()
    {
        static Systick instance;
        return instance;
    }

private:

    Systick()
        : cnt(static_cast<common::time_tick>(0))
    {}

    Systick(Systick&&)      = default;
    Systick(const Systick&) = default;

    Systick& operator = (Systick&&)      = delete;
    Systick& operator = (const Systick&) = delete;

private:

    volatile common::time_tick cnt;

private:

    friend void systick_handle_interrupt(Systick* a_p_this)
    {
        assert(nullptr != a_p_this);
        a_p_this->cnt++;
    }
};

} // namespace hal
} // namespace cml