/*
    Name: system_counter.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <hal/core/system_counter.hpp>

namespace {

using namespace cml::common;

time::tick cnt = 0;

} // namespace ::

namespace cml {
namespace hal {
namespace core {

using namespace cml::common;

time::tick system_counter::get()
{
    return cnt;
}

void system_counter::set(time::tick a_value)
{
    cnt = a_value;
}

void system_counter::reset()
{
    cnt = 0;
}

void system_counter::update(void*)
{
    cnt++;
}

} // namespace core
} // namespace hal
} // namespace cml