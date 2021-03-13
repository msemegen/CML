#pragma once

/*
 *   Name: Interrupt_guard.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// cml
#include <cml/Non_copyable.hpp>

namespace soc {

class Interrupt_guard : private cml::Non_copyable
{
public:
    Interrupt_guard();
    ~Interrupt_guard();

private:
    uint32_t primask;
};

} // namespace soc
