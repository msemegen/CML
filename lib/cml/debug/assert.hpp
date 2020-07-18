#pragma once

/*
    Name: assert.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

namespace cml {
namespace debug {

struct assert
{
    struct Halt
    {
        using Function = void(*)(void* a_p_user_data);

        Function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    struct Print
    {
        using Function = void(*)(void* a_p_user_data,
                                 const char* a_p_file,
                                 uint32_t a_line,
                                 const char* a_p_expression);

        Function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    static void register_halt(const Halt& a_callback);
    static void register_print(const Print& a_callback);
    static void trap(const char* a_p_file, uint32_t a_line, const char* a_p_expression);

    assert()              = delete;
    assert(const assert&) = delete;
    assert(assert&&)      = delete;
    ~assert()             = default;

    assert& operator = (const assert&) = delete;
    assert& operator = (assert&&)      = delete;
};

} // namespace debug
} // namespace cml

#ifdef CML_ASSERT
#define assert(expression) (false == (expression) ? cml::debug::assert::trap(__FILE__,                        \
                                                                             static_cast<uint32_t>(__LINE__), \
                                                                             #expression)                     \
                                                  : static_cast<void>(0))
#endif

#ifndef CML_ASSERT
#define assert(expression)
#endif