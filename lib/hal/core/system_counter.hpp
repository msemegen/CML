#pragma once

/*
    Name: system_counter.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/time.hpp>

namespace cml {
namespace hal {
namespace core {

class system_counter
{
public:

    static common::time::tick get();
    static void set(common::time::tick a_value);
    static void reset();

    static void update(void* a_p_user_data);

private:

    system_counter()                      = delete;
    system_counter(system_counter&&)      = delete;
    system_counter(const system_counter&) = delete;
    ~system_counter()                     = default;

    system_counter& operator = (system_counter&&)      = delete;
    system_counter& operator = (const system_counter&) = delete;
};

} // namespace core
} // namespace hal
} // namespace cml