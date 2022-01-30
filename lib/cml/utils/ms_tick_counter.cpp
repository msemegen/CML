/*
 *   Name: ms_tick_counter.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <cml/utils/ms_tick_counter.hpp>

namespace {
volatile std::uint32_t cnt = 0;
} // namespace

namespace cml {
namespace utils {
std::uint32_t ms_tick_counter::get()
{
    return cnt;
}

void ms_tick_counter::update()
{
    cnt++;
}

void ms_tick_counter::update(void*)
{
    cnt++;
}
} // namespace utils
} // namespace cml