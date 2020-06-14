#pragma once

/*
    Name: integer.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

namespace cml {
namespace common {

static_assert(1 == sizeof(signed char));
static_assert(2 == sizeof(signed short int));
static_assert(4 == sizeof(signed long int));
static_assert(8 == sizeof(signed long long int));

static_assert(1 == sizeof(unsigned char));
static_assert(2 == sizeof(unsigned short int));
static_assert(4 == sizeof(unsigned long int));
static_assert(8 == sizeof(unsigned long long int));

using int8  = signed char;
using int16 = signed short int;
using int32 = signed long int;
using int64 = signed long long int;

using uint8  = unsigned char;
using uint16 = unsigned short int;
using uint32 = unsigned long int;
using uint64 = unsigned long long int;

using byte = uint8;

} // namespace common
} // namespace cml