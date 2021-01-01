/*
    Name: system_timer.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <soc/system_timer.hpp>

namespace {

using namespace cml;

volatile time::tick cnt = 0;

} // namespace

namespace soc {

using namespace cml;

time::tick system_timer::get()
{
    return cnt;
}

void system_timer::update()
{
    cnt++;
}

} // namespace soc