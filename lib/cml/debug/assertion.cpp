/*
 *   Name: assertion.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <cml/debug/assertion.hpp>

namespace {

using namespace cml::debug;

assertion::Halt_hadler halt;
assertion::Print_handler print;

} // namespace

namespace cml {
namespace debug {

void assertion::register_halt(const Halt_hadler& a_handler)
{
    halt = a_handler;
}

void assertion::register_print(const Print_handler& a_handler)
{
    print = a_handler;
}

void assertion::trap(const char* a_p_file, uint32_t a_line, const char* a_p_expression)
{
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