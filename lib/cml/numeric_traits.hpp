#pragma once

/*
    Name: numeric_traits.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

namespace cml {

template<typename Type_t> class numeric_traits
{
public:
    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator=(numeric_traits&&) = delete;
    numeric_traits& operator=(const numeric_traits&) = delete;

    static constexpr Type_t get_min()
    {
        return Type_t();
    }

    static constexpr Type_t get_max()
    {
        return Type_t();
    }

public:
    static constexpr bool is_signed   = false;
    static constexpr bool is_unsigned = false;
};

template<> class numeric_traits<uint8_t>
{
public:
    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator=(numeric_traits&&) = delete;
    numeric_traits& operator=(const numeric_traits&) = delete;

    static constexpr uint8_t get_min()
    {
        static_assert(1 == sizeof(uint8_t));
        return static_cast<uint8_t>(0x0U);
    }

    static constexpr uint8_t get_max()
    {
        static_assert(1 == sizeof(uint8_t));
        return static_cast<uint8_t>(0xFFU);
    }

public:
    static constexpr bool is_signed   = false;
    static constexpr bool is_unsigned = true;
};

template<> class numeric_traits<int8_t>
{
public:
    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator=(numeric_traits&&) = delete;
    numeric_traits& operator=(const numeric_traits&) = delete;

    static constexpr int8_t get_min()
    {
        static_assert(1 == sizeof(int8_t));
        return static_cast<int8_t>(-0x80);
    }

    static constexpr int8_t get_max()
    {
        static_assert(1 == sizeof(int8_t));
        return static_cast<int8_t>(0x7F);
    }

public:
    static constexpr bool is_signed   = true;
    static constexpr bool is_unsigned = false;
};

template<> class numeric_traits<uint16_t>
{
public:
    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator=(numeric_traits&&) = delete;
    numeric_traits& operator=(const numeric_traits&) = delete;

    static uint16_t get_min()
    {
        static_assert(2 == sizeof(uint16_t));
        return static_cast<uint16_t>(0x0U);
    }

    static uint16_t get_max()
    {
        static_assert(2 == sizeof(uint16_t));
        return static_cast<uint16_t>(0xFFFFU);
    }

public:
    static constexpr bool is_signed   = false;
    static constexpr bool is_unsigned = true;
};

template<> class numeric_traits<int16_t>
{
public:
    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator=(numeric_traits&&) = delete;
    numeric_traits& operator=(const numeric_traits&) = delete;

    static constexpr int16_t get_min()
    {
        static_assert(2 == sizeof(int16_t));
        return static_cast<int16_t>(-0x8000);
    }

    static constexpr int16_t get_max()
    {
        static_assert(2 == sizeof(int16_t));
        return static_cast<int16_t>(0x7FFF);
    }

public:
    static constexpr bool is_signed   = true;
    static constexpr bool is_unsigned = false;
};

template<> class numeric_traits<uint32_t>
{
public:
    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator=(numeric_traits&&) = delete;
    numeric_traits& operator=(const numeric_traits&) = delete;

    static constexpr uint32_t get_min()
    {
        static_assert(4 == sizeof(uint32_t));
        return static_cast<uint32_t>(0x0U);
    }

    static constexpr uint32_t get_max()
    {
        static_assert(4 == sizeof(uint32_t));
        return static_cast<uint32_t>(0xFFFFFFFFU);
    }

public:
    static constexpr bool is_signed   = false;
    static constexpr bool is_unsigned = true;
};

template<> class numeric_traits<int32_t>
{
public:
    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator=(numeric_traits&&) = delete;
    numeric_traits& operator=(const numeric_traits&) = delete;

    static constexpr int32_t get_min()
    {
        static_assert(4 == sizeof(int32_t));
        return static_cast<int32_t>(-0x80000000);
    }

    static constexpr int32_t get_max()
    {
        static_assert(4 == sizeof(int32_t));
        return static_cast<int32_t>(0x7FFFFFFF);
    }

public:
    static constexpr bool is_signed   = true;
    static constexpr bool is_unsigned = false;
};

template<> class numeric_traits<uint64_t>
{
public:
    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator=(numeric_traits&&) = delete;
    numeric_traits& operator=(const numeric_traits&) = delete;

    static constexpr uint64_t get_min()
    {
        static_assert(8 == sizeof(uint64_t));
        return static_cast<uint64_t>(0x0ULL);
    }

    static constexpr uint64_t get_max()
    {
        static_assert(8 == sizeof(uint64_t));
        return static_cast<uint64_t>(0xFFFFFFFFFFFFFFFFULL);
    }

public:
    static constexpr bool is_signed   = false;
    static constexpr bool is_unsigned = true;
};

template<> class numeric_traits<int64_t>
{
public:
    numeric_traits()                      = delete;
    numeric_traits(numeric_traits&&)      = delete;
    numeric_traits(const numeric_traits&) = delete;
    ~numeric_traits()                     = delete;

    numeric_traits& operator=(numeric_traits&&) = delete;
    numeric_traits& operator=(const numeric_traits&) = delete;

    static constexpr int64_t get_min()
    {
        static_assert(8 == sizeof(int64_t));
        return static_cast<int64_t>(-0x8000000000000000LL);
    }

    static constexpr int64_t get_max()
    {
        static_assert(8 == sizeof(int64_t));
        return static_cast<int64_t>(0x7FFFFFFFFFFFFFFFLL);
    }

public:
    static constexpr bool is_signed   = true;
    static constexpr bool is_unsigned = false;
};

} // namespace cml