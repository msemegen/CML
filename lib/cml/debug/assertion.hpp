#pragma once

/*
 *   Name: assertion.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// cml
#include <cml/Non_constructible.hpp>

namespace cml {
namespace debug {
class assertion : private cml::Non_constructible
{
public:
#if defined(M4)
    enum class Trap_enter_mode : std::uint32_t
    {
        disabled,
        enabled
    };
#endif

    struct Halt_hadler
    {
        using Function = void (*)(void* a_p_user_data);

        Function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    struct Print_handler
    {
        using Function = void (*)(const char* a_p_file,
                                  uint32_t a_line,
                                  const char* a_p_expression,
                                  void* a_p_user_data);

        Function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    static void enable(const Halt_hadler& a_halt,
                       const Print_handler& a_print
#if defined(M4)
                       ,
                       Trap_enter_mode a_trap_enter_mode
#endif
    );
    static void disable();

    static void trap(const char* a_p_file, uint32_t a_line, const char* a_p_expression);
};
} // namespace debug
} // namespace cml

#ifdef CML_ASSERT_ENABLED
#define cml_assert(expression)                                                                                     \
    (false == (expression) ? cml::debug::assertion::trap(__FILE__, static_cast<uint32_t>(__LINE__), #expression) : \
                             static_cast<void>(0))
#endif

#ifndef CML_ASSERT_ENABLED
#define cml_assert(expression)
#endif