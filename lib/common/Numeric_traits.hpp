#pragma once

/*
    Name: Numeric_traits.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/assert.hpp>
#include <common/integer.hpp>

namespace cml {
namespace common {

template<typename type>
class Numeric_traits
{
public:

    Numeric_traits()                      = delete;
    Numeric_traits(Numeric_traits&&)      = delete;
    Numeric_traits(const Numeric_traits&) = delete;
    ~Numeric_traits()                     = delete;

    Numeric_traits& operator = (Numeric_traits&&)      = delete;
    Numeric_traits& operator = (const Numeric_traits&) = delete;

    static constexpr type get_min()
    {
        assert(false);
        return type();
    }

    static constexpr type get_max()
    {
        assert(false);
        return type();
    }
};

template<>
class Numeric_traits<uint8>
{
public:

    Numeric_traits()                      = delete;
    Numeric_traits(Numeric_traits&&)      = delete;
    Numeric_traits(const Numeric_traits&) = delete;
    ~Numeric_traits()                     = delete;

    Numeric_traits& operator = (Numeric_traits&&)      = delete;
    Numeric_traits& operator = (const Numeric_traits&) = delete;

    static constexpr uint8 get_min()
    {
        static_assert(1 == sizeof(uint8));
        return static_cast<uint8>(0x0U);
    }

    static constexpr uint8 get_max()
    {
        static_assert(1 == sizeof(uint8));
        return static_cast<uint8>(0xFFU);
    }
};

template<>
class Numeric_traits<int8>
{
public:

    Numeric_traits()                      = delete;
    Numeric_traits(Numeric_traits&&)      = delete;
    Numeric_traits(const Numeric_traits&) = delete;
    ~Numeric_traits()                     = delete;

    Numeric_traits& operator = (Numeric_traits&&)      = delete;
    Numeric_traits& operator = (const Numeric_traits&) = delete;

    static constexpr int8 get_min()
    {
        static_assert(1 == sizeof(int8));
        return static_cast<int8>(-0x80);
    }

    static constexpr int8 get_max()
    {
        static_assert(1 == sizeof(int8));
        return static_cast<int8>(0x7F);
    }
};

template<>
class Numeric_traits<uint16>
{
public:

    Numeric_traits()                      = delete;
    Numeric_traits(Numeric_traits&&)      = delete;
    Numeric_traits(const Numeric_traits&) = delete;
    ~Numeric_traits()                     = delete;

    Numeric_traits& operator = (Numeric_traits&&)      = delete;
    Numeric_traits& operator = (const Numeric_traits&) = delete;

    static uint16 get_min()
    {
        static_assert(2 == sizeof(uint16));
        return static_cast<uint16>(0x0U);
    }

    static uint16 get_max()
    {
        static_assert(2 == sizeof(uint16));
        return static_cast<uint16>(0xFFFFU);
    }
};

template<>
class Numeric_traits<int16>
{
public:

    Numeric_traits()                      = delete;
    Numeric_traits(Numeric_traits&&)      = delete;
    Numeric_traits(const Numeric_traits&) = delete;
    ~Numeric_traits()                     = delete;

    Numeric_traits& operator = (Numeric_traits&&)      = delete;
    Numeric_traits& operator = (const Numeric_traits&) = delete;

    static constexpr int16 get_min()
    {
        static_assert(2 == sizeof(int16));
        return static_cast<int16>(-0x8000);
    }

    static constexpr int16 get_max()
    {
        static_assert(2 == sizeof(int16));
        return static_cast<int16>(0x7FFF);
    }
};

template<>
class Numeric_traits<uint32>
{
public:

    Numeric_traits()                      = delete;
    Numeric_traits(Numeric_traits&&)      = delete;
    Numeric_traits(const Numeric_traits&) = delete;
    ~Numeric_traits()                     = delete;

    Numeric_traits& operator = (Numeric_traits&&)      = delete;
    Numeric_traits& operator = (const Numeric_traits&) = delete;

    static constexpr uint32 get_min()
    {
        static_assert(4 == sizeof(uint32));
        return static_cast<uint32>(0x0U);
    }

    static constexpr uint32 get_max()
    {
        static_assert(4 == sizeof(uint32));
        return static_cast<uint32>(0xFFFFFFFFU);
    }
};

template<>
class Numeric_traits<int32>
{
public:

    Numeric_traits()                      = delete;
    Numeric_traits(Numeric_traits&&)      = delete;
    Numeric_traits(const Numeric_traits&) = delete;
    ~Numeric_traits()                     = delete;

    Numeric_traits& operator = (Numeric_traits&&)      = delete;
    Numeric_traits& operator = (const Numeric_traits&) = delete;

    static constexpr int32 get_min()
    {
        static_assert(4 == sizeof(int32));
        return static_cast<int32>(-0x80000000);
    }

    static constexpr int32 get_max()
    {
        static_assert(4 == sizeof(int32));
        return static_cast<int32>(0x7FFFFFFF);
    }
};

template<>
class Numeric_traits<uint64>
{
public:

    Numeric_traits()                      = delete;
    Numeric_traits(Numeric_traits&&)      = delete;
    Numeric_traits(const Numeric_traits&) = delete;
    ~Numeric_traits()                     = delete;

    Numeric_traits& operator = (Numeric_traits&&)      = delete;
    Numeric_traits& operator = (const Numeric_traits&) = delete;

    static constexpr uint64 get_min()
    {
        static_assert(8 == sizeof(uint64));
        return static_cast<uint64>(0x0ULL);
    }

    static constexpr uint64 get_max()
    {
        static_assert(8 == sizeof(uint64));
        return static_cast<uint64>(0xFFFFFFFFFFFFFFFFULL);
    }
};

template<>
class Numeric_traits<int64>
{
public:

    Numeric_traits()                      = delete;
    Numeric_traits(Numeric_traits&&)      = delete;
    Numeric_traits(const Numeric_traits&) = delete;
    ~Numeric_traits()                     = delete;

    Numeric_traits& operator = (Numeric_traits&&)      = delete;
    Numeric_traits& operator = (const Numeric_traits&) = delete;

    static constexpr int64 get_min()
    {
        static_assert(8 == sizeof(int64));
        return static_cast<int64>(-0x8000000000000000LL);
    }

    static constexpr int64 get_max()
    {
        static_assert(8 == sizeof(int64));
        return static_cast<int64>(0x7FFFFFFFFFFFFFFFLL);
    }
};

} // namespace common
} // namespace cml