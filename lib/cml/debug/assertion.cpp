/*
 *   Name: assertion.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <cml/debug/assertion.hpp>

// soc
#if defined(M4)
#include <cml/hal/mcu.hpp>
#endif

namespace {
using namespace cml::debug;

assertion::Halt_hadler halt;
assertion::Print_handler print;
#if defined(M4)
assertion::Trap_enter_mode trap_enter_mode;
#endif
} // namespace

namespace cml {
namespace debug {
#if defined(M4)
using namespace cml::hal;
#endif

void assertion::enable(const Halt_hadler& a_halt,
                       const Print_handler& a_print
#if defined(M4)
                       ,
                       Trap_enter_mode a_trap_enter_mode
#endif
)
{
    halt            = a_halt;
    print           = a_print;
    trap_enter_mode = a_trap_enter_mode;
}

void assertion::disable()
{
    halt  = { nullptr, nullptr };
    print = { nullptr, nullptr };
}

void assertion::trap(const char* a_p_file, uint32_t a_line, const char* a_p_expression)
{
#if defined(M4)
    if (Trap_enter_mode::enabled == trap_enter_mode && true == mcu::is_in_debug_mode())
    {
        __BKPT(0);
    }
#endif
    if (nullptr != print.p_function)
    {
        print.p_function(a_p_file, a_line, a_p_expression, print.p_user_data);
    }

    if (nullptr != halt.p_function)
    {
        halt.p_function(halt.p_user_data);
    }
}
} // namespace debug
} // namespace cml