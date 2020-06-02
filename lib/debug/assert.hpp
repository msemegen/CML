#pragma once

/*
    Name: assert.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>

namespace cml {
namespace debug {

struct assert
{
    struct Halt_callback
    {
        using Function = void(*)(void* a_p_user_data);

        Function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    struct Print_callback
    {
        using Function = void(*)(void* a_p_user_data,
                                 const char* a_p_file,
                                 common::uint32 a_line,
                                 const char* a_p_expression);

        Function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    static void register_callback(const Halt_callback& a_callback);
    static void register_callback(const Print_callback& a_callback);
    static void trap(const char* a_p_file, common::uint32 a_line, const char* a_p_expression);

    assert()              = delete;
    assert(const assert&) = delete;
    assert(assert&&)      = delete;
    ~assert()             = default;

    assert& operator = (const assert&) = delete;
    assert& operator = (assert&&)      = delete;
};

} // namespace debug
} // namespace cml

#ifdef CML_DEBUG
#define assert(expression) (false == (expression) ? cml::debug::assert::trap(__FILE__, \
                                                                             static_cast<cml::common::uint32>(__LINE__),\
                                                                             #expression) \
                                                  : static_cast<void>(0))
#endif

#ifndef CML_DEBUG
#define assert(expression)
#endif