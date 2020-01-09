#pragma once

/*
    Name: assert.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>

namespace cml {
namespace common {

class assert
{
public:

    struct Halt_callback
    {
        using function = void(*)(void* a_p_user_data);

        function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    struct Print_callback
    {
        using function = void(*)(void* a_p_user_data,
                                 const char* a_p_file,
                                 int32 a_line,
                                 const char* a_p_expression);

        function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    void register_callback(const Halt_callback& a_callback)
    {
        this->halt = a_callback;
    }

    void register_callback(const Print_callback& a_callback)
    {
        this->print = a_callback;
    }

    void trap(const char* a_p_file, int32 a_line, const char* a_p_expression)
    {
        if (nullptr != this->print.p_function)
        {
            this->print.p_function(this->print.p_user_data, a_p_file, a_line, a_p_expression);
        }

        if (nullptr != this->halt.p_function)
        {
            this->halt.p_function(this->halt.p_user_data);
        }
    }

    static assert& get_instance()
    {
        static assert instance;
        return instance;
    }

private:

    assert()              = default;
    assert(const assert&) = delete;
    assert(assert&&)      = delete;
    ~assert()             = default;

    assert& operator = (const assert&) = delete;
    assert& operator = (assert&&)      = delete;

private:

    Halt_callback  halt;
    Print_callback print;
};

} // namespace common
} // namespace cml

#ifdef DEBUG
#define assert(expression) (false == (expression) ? cml::common::assert::get_instance().trap(__FILE__,    \
                                                                                             __LINE__,    \
                                                                                             #expression) \
                                                  : static_cast<void>(0))
#endif

#ifndef DEBUG
#define assert(expression)
#endif