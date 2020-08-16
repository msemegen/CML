#pragma once

/*
    Name: interrupt_guard.hpp

    Copyright(c) 2020 Jay Kickliter
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

namespace soc {

/**
 * Provides an RAII style uninterruptible section.
 *
 * `Interrupt_guard` disables interrupts for its entire lifetime
 * so care must be taken to limit it's scope when constructed.
 *
 * # Example
 *
 *     // Interrupts are enabled, but we need to update a
 *     // non-atomic global variable.
 *
 *     {
 *         Interrupt_guard guard;
 *         // interrupts now disabled until next closing brace.
 *         some_large_global = SomeLargeStruct();
 *     }   // guard's destructor called here
 *     // Interrupts are enabled.
 */
class Interrupt_guard
{
public:
    Interrupt_guard();
    ~Interrupt_guard();

    Interrupt_guard(Interrupt_guard&&)      = delete;
    Interrupt_guard(const Interrupt_guard&) = delete;
    Interrupt_guard& operator=(Interrupt_guard&&) = delete;
    Interrupt_guard& operator=(const Interrupt_guard&) = delete;

private:
    uint32_t primask;
};

} // namespace soc
