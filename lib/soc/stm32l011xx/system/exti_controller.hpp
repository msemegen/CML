#pragma once

/*
    Name: exti_controller.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//std
#include <cstdint>

//soc
#include <soc/stm32l011xx/peripherals/GPIO.hpp>

namespace soc {
namespace stm32l011xx {
namespace system {

class exti_controller
{
public:

    enum class Interrupt_mode : uint32_t
    {
        rising  = 0x1,
        falling = 0x2,
    };

    struct Callback
    {
        using Function = bool(*)(peripherals::Input_pin::Level a_level, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:

    exti_controller()                       = delete;
    exti_controller(exti_controller&&)      = delete;
    exti_controller(const exti_controller&) = delete;

    exti_controller& operator = (exti_controller&&)      = delete;
    exti_controller& operator = (const exti_controller&) = delete;

    static void enable(uint32_t a_priority);
    static void disable();

    static void register_callback(peripherals::Input_pin* a_p_pin,
                                  Interrupt_mode a_mode,
                                  const Callback& a_callback);

    static void unregister_callback(const peripherals::Input_pin& a_pin);
};

constexpr exti_controller::Interrupt_mode operator | (exti_controller::Interrupt_mode a_f1,
                                                      exti_controller::Interrupt_mode a_f2)
{
    return static_cast<exti_controller::Interrupt_mode>(static_cast<uint32_t>(a_f1) |
                                                        static_cast<uint32_t>(a_f2));
}

constexpr exti_controller::Interrupt_mode operator & (exti_controller::Interrupt_mode a_f1,
                                                      exti_controller::Interrupt_mode a_f2)
{
    return static_cast<exti_controller::Interrupt_mode>(static_cast<uint32_t>(a_f1) &
                                                        static_cast<uint32_t>(a_f2));
}

constexpr exti_controller::Interrupt_mode operator |= (exti_controller::Interrupt_mode& a_f1,
                                                       exti_controller::Interrupt_mode a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

} // namespace system
} // namespace stm32l011xx
} // namespace soc