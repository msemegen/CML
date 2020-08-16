/*
    Name: system_counter.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <soc/counter.hpp>

namespace {

using namespace cml;

time::tick cnt = 0;

} // namespace

namespace soc {

using namespace cml;

time::tick counter::get()
{
    return cnt;
}

void counter::set(time::tick a_value)
{
    cnt = a_value;
}

void counter::reset()
{
    cnt = 0;
}

void counter::update(void*)
{
    cnt++;
}

} // namespace soc