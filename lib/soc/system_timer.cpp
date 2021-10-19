/*
 *   Name: system_timer.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/system_timer.hpp>

namespace {

volatile std::uint32_t cnt = 0;

} // namespace

namespace soc {

std::uint32_t system_timer::get()
{
    return cnt;
}

void system_timer::update()
{
    cnt++;
}

void system_timer::update(void*)
{
    cnt++;
}

} // namespace soc