#pragma once

/*
    Name: integer.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

namespace cml {
namespace common {

static_assert(1 == sizeof(char));
static_assert(2 == sizeof(short int));
static_assert(4 == sizeof(int));
static_assert(8 == sizeof(long long int));

static_assert(1 == sizeof(unsigned char));
static_assert(2 == sizeof(unsigned short int));
static_assert(4 == sizeof(unsigned int));
static_assert(8 == sizeof(unsigned long long int));

using int8  = char;
using int16 = short int;
using int32 = int;
using int64 = long long int;

using uint8  = unsigned char;
using uint16 = unsigned short int;
using uint32 = unsigned int;
using uint64 = unsigned long long int;

using byte = uint8;

} // namespace common
} // namespace cml