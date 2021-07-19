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
#ifdef ARM_CORTEX_M4
#include <soc/m4/nvic.hpp>
#endif

// cml
#include <cml/Non_copyable.hpp>

namespace soc {

class Interrupt_guard : private cml::Non_copyable
{
public:
    Interrupt_guard()
    {
#ifdef ARM_CORTEX_M4
        m4::nvic::set_mode(m4::nvic::Mode::disabled);
#else
        static_assert(false, "Not implemented architecture");
#endif
    }

    ~Interrupt_guard()
    {
#ifdef ARM_CORTEX_M4
        m4::nvic::set_mode(m4::nvic::Mode::enabled);
#else
        static_assert(false, "Not implemented architecture");
#endif
    }
};

} // namespace soc
