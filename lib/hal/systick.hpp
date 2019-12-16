#pragma once

/*
    Name: systick.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <common/time_tick.hpp>

namespace cml {
namespace hal {

class c_systick
{
public:

    void enable();
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

    static c_systick& get_instance()
    {
        static c_systick instance;
        return instance;
    }

private:

    c_systick()
        : cnt(static_cast<common::time_tick>(0))
    {}

    c_systick(c_systick&&)      = default;
    c_systick(const c_systick&) = default;

    c_systick& operator = (c_systick&&)      = delete;
    c_systick& operator = (const c_systick&) = delete;

    volatile common::time_tick cnt;

    friend class c_interrupt_handler;
};

} // namespace hal
} // namespace cml