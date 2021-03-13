#pragma once

/*
 *   Name: assertion.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

namespace cml {
namespace debug {

struct assertion
{
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

    static void register_halt(const Halt_hadler& a_handler);
    static void register_print(const Print_handler& a_handler);
    static void trap(const char* a_p_file, uint32_t a_line, const char* a_p_expression);

    assertion()                 = delete;
    assertion(const assertion&) = delete;
    assertion(assertion&&)      = delete;
    ~assertion()                = default;

    assertion& operator=(const assertion&) = delete;
    assertion& operator=(assertion&&) = delete;
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