#pragma once

/*
    Name: counter.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <cml/time.hpp>

namespace soc {

class counter
{
public:

    static cml::time::tick get();
    static void set(cml::time::tick a_value);
    static void reset();

    static void update(void* a_p_user_data);

private:

    counter()               = delete;
    counter(counter&&)      = delete;
    counter(const counter&) = delete;
    ~counter()              = default;

    counter& operator = (counter&&)      = delete;
    counter& operator = (const counter&) = delete;
};

} // namespace soc
