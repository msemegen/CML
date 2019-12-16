//externals
#define NULL 0
#include <gtest.h>

//mcl
#include <hal/cortexm/stm32l452xx/gpio.hpp>

TEST(enable, gpio)
{
    using namespace cml::hal::cortexm::stm32l452xx;

    gpio gpio_a(gpio::periph::a);
    gpio_a.enable();
}