#pragma once

/*
    Name: compiler.hpp

    Copyright(c) 2019 - 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef __GNUG__
#define force_align(v) __attribute__(aligned(v))
#define force_pack __attribute__((__packed__))
#define force_inline __attribute__((always_inline))
#endif