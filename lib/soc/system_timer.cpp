/*
 *   Name: system_timer.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// this
#include <soc/system_timer.hpp>

namespace {

volatile uint32_t cnt = 0;

} // namespace

namespace soc {

uint32_t system_timer::get()
{
    return cnt;
}

void system_timer::update()
{
    cnt++;
}

} // namespace soc