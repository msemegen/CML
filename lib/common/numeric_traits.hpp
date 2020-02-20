#pragma once

/*
    Name: numeric_traits.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <common/integer.hpp>
#include <debug/assert.hpp>

namespace cml {
namespace common {

template<typename type>
class numeric_traits
{
public:

    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator = (numeric_traits&&)      = delete;
    numeric_traits& operator = (const numeric_traits&) = delete;

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
class numeric_traits<uint8>
{
public:

    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator = (numeric_traits&&)      = delete;
    numeric_traits& operator = (const numeric_traits&) = delete;

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
class numeric_traits<int8>
{
public:

    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator = (numeric_traits&&)      = delete;
    numeric_traits& operator = (const numeric_traits&) = delete;

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
class numeric_traits<uint16>
{
public:

    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator = (numeric_traits&&)      = delete;
    numeric_traits& operator = (const numeric_traits&) = delete;

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
class numeric_traits<int16>
{
public:

    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator = (numeric_traits&&)      = delete;
    numeric_traits& operator = (const numeric_traits&) = delete;

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
class numeric_traits<uint32>
{
public:

    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator = (numeric_traits&&)      = delete;
    numeric_traits& operator = (const numeric_traits&) = delete;

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
class numeric_traits<int32>
{
public:

    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator = (numeric_traits&&)      = delete;
    numeric_traits& operator = (const numeric_traits&) = delete;

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
class numeric_traits<uint64>
{
public:

    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator = (numeric_traits&&)      = delete;
    numeric_traits& operator = (const numeric_traits&) = delete;

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
class numeric_traits<int64>
{
public:

    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator = (numeric_traits&&)      = delete;
    numeric_traits& operator = (const numeric_traits&) = delete;

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