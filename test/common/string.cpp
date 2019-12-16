#define NULL 0
//
#include <gtest.h>

//cml
#include <common/string.hpp>

TEST(default_construction, string)
{
    using namespace cml::common;

    string<10> str;

    str.get_length();
    str.get_capacity();
    str.get_c_string();
}