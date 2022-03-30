#pragma once

/*
 *   Name: Duration.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <ratio>
#include <type_traits>

namespace cml {
template<typename Value_t, Value_t factor_t> class Duration
{
public:
    using Value_type                    = Value_t;
    static constexpr Value_t factor     = factor_t;
    constexpr Duration()                = default;
    constexpr Duration(Duration&&)      = default;
    constexpr Duration(const Duration&) = default;
    constexpr Duration(Value_t a_v)
        : v(a_v)
    {
    }
    template<Value_t rhs_factor, std::enable_if_t<(factor_t < rhs_factor), bool> = true>
    constexpr Duration(const Duration<Value_t, rhs_factor>& a_v)
        : v(a_v.get() * rhs_factor)
    {
    }
    template<Value_t rhs_factor, std::enable_if_t<(factor_t < rhs_factor), bool> = true>
    explicit constexpr Duration(Duration<Value_t, rhs_factor>&& a_v)
        : v(a_v.get() * rhs_factor)
    {
    }

    constexpr Duration& operator=(const Duration&) = default;
    constexpr Duration& operator=(Duration&&) = default;
    template<Value_t rhs_factor> constexpr bool operator==(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t == a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator!=(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t != a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator>=(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t >= a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator<=(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t <= a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator>(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t > a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator<(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t < a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor, std::enable_if_t<(factor_t < rhs_factor), bool> = true>
    constexpr Duration<Value_t, factor>& operator=(const Duration<Value_t, rhs_factor>& a_rhs)
    {
        this->v = a_rhs.v * rhs_factor;
        return *this;
    }
    template<Value_t rhs_factor, std::enable_if_t<(factor_t < rhs_factor), bool> = true>
    constexpr Duration<Value_t, factor_t>& operator=(Duration<Value_t, rhs_factor>&& a_rhs)
    {
        this->v = a_rhs.v * rhs_factor;
        return *this;
    }
    Duration& operator++()
    {
        this->v++;
        return *this;
    }
    Duration operator++(int)
    {
        return Duration(this->v++);
    }
    Duration& operator--()
    {
        this->v--;
        return *this;
    }
    Duration operator--(int)
    {
        return Duration(this->v--);
    }
    Duration& operator+=(const Duration& a_rhs)
    {
        this->v += a_rhs.v;
        return *this;
    }
    Duration& operator-=(const Duration& a_rhs)
    {
        this->v -= a_rhs.v;
        return *this;
    }
    Duration& operator*=(Value_t a_rhs)
    {
        this->v *= a_rhs;
        return *this;
    }
    Duration& operator/=(Value_t a_rhs)
    {
        this->v /= a_rhs;
        return *this;
    }
    Duration& operator%=(Value_t a_rhs)
    {
        this->v %= a_rhs;
        return *this;
    }
    Duration& operator%=(const Duration& a_rhs)
    {
        this->v %= a_rhs.v;
        return *this;
    }

    Value_t get() const
    {
        return this->v;
    }

    template<typename Type_t, std::enable_if_t<(factor_t > Type_t::factor), bool> = true>
    Duration<Value_t, Type_t::factor> convert()
    {
        return { this->v * Type_t::factor };
    }

private:
    Value_t v = static_cast<Value_t>(0);
};

using Microseconds = Duration<std::uint64_t, 1u>;
using Milliseconds = Duration<std::uint64_t, 1000u>;
using Seconds      = Duration<std::uint64_t, 1000000u>;

inline Microseconds operator-(Microseconds a_lhs, Microseconds a_rhs)
{
    return { a_lhs.get() - a_rhs.get() };
}
inline Microseconds operator-(Microseconds a_lhs, Milliseconds a_rhs)
{
    return { a_lhs - a_rhs.convert<Microseconds>() };
}
inline Microseconds operator-(Milliseconds a_lhs, Microseconds a_rhs)
{
    return { a_lhs.convert<Microseconds>() - a_rhs };
}
inline Milliseconds operator-(Milliseconds a_lhs, Milliseconds a_rhs)
{
    return { a_lhs.get() - a_rhs.get() };
}
inline Milliseconds operator-(Seconds a_lhs, Milliseconds a_rhs)
{
    return { a_lhs.convert<Milliseconds>() - a_rhs };
}
inline Milliseconds operator-(Milliseconds a_lhs, Seconds a_rhs)
{
    return { a_lhs - a_rhs.convert<Milliseconds>() };
}
inline Seconds operator-(Seconds a_lhs, Seconds a_rhs)
{
    return { a_lhs.get() - a_rhs.get() };
}

inline Microseconds operator+(Microseconds a_lhs, Microseconds a_rhs)
{
    return { a_lhs.get() + a_rhs.get() };
}
inline Microseconds operator+(Microseconds a_lhs, Milliseconds a_rhs)
{
    return { a_lhs + a_rhs.convert<Microseconds>() };
}
inline Microseconds operator+(Milliseconds a_lhs, Microseconds a_rhs)
{
    return { a_lhs.convert<Microseconds>() + a_rhs };
}
inline Milliseconds operator+(Milliseconds a_lhs, Milliseconds a_rhs)
{
    return { a_lhs.get() + a_rhs.get() };
}
inline Milliseconds operator+(Seconds a_lhs, Milliseconds a_rhs)
{
    return { a_lhs.convert<Milliseconds>() + a_rhs };
}
inline Milliseconds operator+(Milliseconds a_lhs, Seconds a_rhs)
{
    return { a_lhs + a_rhs.convert<Milliseconds>() };
}
inline Seconds operator+(Seconds a_lhs, Seconds a_rhs)
{
    return { a_lhs.get() + a_rhs.get() };
}

inline Microseconds operator*(Microseconds a_lhs, Microseconds::Value_type a_rhs)
{
    return { a_lhs.get() * a_rhs };
}
inline Microseconds operator*(Microseconds::Value_type a_lhs, Microseconds a_rhs)
{
    return { a_lhs * a_rhs.get() };
}

inline Milliseconds operator*(Milliseconds a_lhs, Milliseconds::Value_type a_rhs)
{
    return { a_lhs.get() * a_rhs };
}
inline Milliseconds operator*(Milliseconds::Value_type a_lhs, Milliseconds a_rhs)
{
    return { a_lhs * a_rhs.get() };
}

inline Seconds operator*(Seconds a_lhs, Seconds::Value_type a_rhs)
{
    return { a_lhs.get() * a_rhs };
}
inline Seconds operator*(Seconds::Value_type a_lhs, Seconds a_rhs)
{
    return { a_lhs * a_rhs.get() };
}
} // namespace cml

inline cml::Microseconds operator"" _us(std::uint64_t a_value)
{
    return { a_value };
}
inline cml::Milliseconds operator"" _ms(std::uint64_t a_value)
{
    return { a_value };
}
inline cml::Seconds operator"" _s(std::uint64_t a_value)
{
    return { a_value };
}