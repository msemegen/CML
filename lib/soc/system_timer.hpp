#pragma once

/*
    Name: system_timer.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// cml
#include <cml/time.hpp>

namespace soc {

class system_timer
{
public:
    static cml::time::tick get();
    static void update();

private:
    system_timer()                    = delete;
    system_timer(system_timer&&)      = delete;
    system_timer(const system_timer&) = delete;
    ~system_timer()                   = default;

    system_timer& operator=(system_timer&&) = delete;
    system_timer& operator=(const system_timer&) = delete;
};

} // namespace soc
