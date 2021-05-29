#pragma once

/*
 *   Name: Basic_timer.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

class Basic_timer : private cml::Non_copyable
{
public:
    enum class Id
    {
        _6,
#if defined(STM32L443xx)
        _7
#endif
    };

public:
    Basic_timer(Id a_id)
        : id(a_id)
    {
    }

    ~Basic_timer()
    {
        this->disable();
    }

    void enable();
    void disable();

    void start();
    void stop();

private:
    Id id;
};

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc
