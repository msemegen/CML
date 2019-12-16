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

class c_assert
{
public:

    struct s_halt_callback
    {
        using function = void(*)(void* a_p_user_data);

        function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    struct s_print_callback
    {
        using function = void(*)(void* a_p_user_data,
                                 const char* a_p_file,
                                 int32 a_line,
                                 const char* a_p_expression);

        function p_function = nullptr;
        void* p_user_data   = nullptr;
    };

    void register_callback(const s_halt_callback& a_callback)
    {
        this->halt = a_callback;
    }

    void register_callback(const s_print_callback& a_callback)
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

    static c_assert& get_instance()
    {
        static c_assert instance;
        return instance;
    }

private:

    c_assert()                = default;
    c_assert(const c_assert&) = delete;
    c_assert(c_assert&&)      = delete;
    ~c_assert()               = default;

    c_assert& operator = (const c_assert&) = delete;
    c_assert& operator = (c_assert&&)      = delete;

private:

    s_halt_callback halt;
    s_print_callback print;
};

} // namespace common
} // namespace cml

#ifdef DEBUG
#define _assert(expression) (false == (expression) ? cml::common::c_assert::get_instance().trap(__FILE__,    \
                                                                                                __LINE__,    \
                                                                                                #expression) \
                                                   : static_cast<void>(0))
#endif

#ifndef DEBUG
#define _assert(expression)
#endif