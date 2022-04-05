#pragma once

/*
 *   Name: Interrupt_guard.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// soc
#include <soc/m4/nvic.hpp>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {
namespace m4 {
class Interrupt_guard : private cml::Non_copyable
{
public:
    Interrupt_guard()
    {
        m4::nvic::set_mode(m4::nvic::Mode::disabled);
    }

    ~Interrupt_guard()
    {
        m4::nvic::set_mode(m4::nvic::Mode::enabled);
    }
};
} // namespace m4
} // namespace soc
