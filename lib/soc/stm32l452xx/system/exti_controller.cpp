/*
    Name: exti_controller.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <soc/stm32l452xx/system/exti_controller.hpp>

//soc
#include <soc/stm32l452xx/peripherals/GPIO.hpp>
#ifdef CML_ASSERT
#include <soc/stm32l452xx/mcu.hpp>
#endif

//cml
#include <cml/debug/assert.hpp>

namespace {

using namespace soc::stm32l452xx::peripherals;
using namespace soc::stm32l452xx::system;

struct Handler
{
    exti_controller::Callback callback;
    Input_pin const* p_pin = nullptr;
};

Handler handlers[16];

} // namespace ::

extern "C"
{

using namespace cml;

bool interrupt_handler(uint32 a_pr1, uint32 a_index)
{
    assert(nullptr != handlers[a_index].callback.function);

    if (true == is_bit(a_pr1, a_index))
    {
        return handlers[a_index].callback.function(handlers[a_index].p_pin->get_level(),
                                                   handlers[a_index].callback.p_user_data);
    }

    return false;
}

void EXTI0_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 0))
    {
        set_bit(&(EXTI->PR1), 0);
    }
}

void EXTI1_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 1))
    {
        set_bit(&(EXTI->PR1), 1);
    }
}

void EXTI2_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 2))
    {
        set_bit(&(EXTI->PR1), 2);
    }
}

void EXTI3_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 3))
    {
        set_bit(&(EXTI->PR1), 3);
    }
}

void EXTI4_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 4))
    {
        set_bit(&(EXTI->PR1), 4);
    }
}

void EXTI9_5_IRQHandler()
{
    for (uint32 i = 5u; i <= 9u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i))
        {
            set_bit(&(EXTI->PR1), i);
        }
    }
}

void EXTI15_10_IRQHandler()
{
    for (uint32 i = 10u; i <= 15u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i))
        {
            set_bit(&(EXTI->PR1), i);
        }
    }
}

} // extern "C"

namespace soc {
namespace stm32l452xx {
namespace system {

using namespace cml;
using namespace soc::stm32l452xx::peripherals;

void exti_controller::enable(uint32 a_priority)
{
    NVIC_SetPriority(EXTI0_IRQn,     a_priority);
    NVIC_SetPriority(EXTI1_IRQn,     a_priority);
    NVIC_SetPriority(EXTI2_IRQn,     a_priority);
    NVIC_SetPriority(EXTI3_IRQn,     a_priority);
    NVIC_SetPriority(EXTI4_IRQn,     a_priority);
    NVIC_SetPriority(EXTI9_5_IRQn,   a_priority);
    NVIC_SetPriority(EXTI15_10_IRQn, a_priority);

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void exti_controller::disable()
{
     NVIC_DisableIRQ(EXTI0_IRQn);
     NVIC_DisableIRQ(EXTI1_IRQn);
     NVIC_DisableIRQ(EXTI2_IRQn);
     NVIC_DisableIRQ(EXTI3_IRQn);
     NVIC_DisableIRQ(EXTI4_IRQn);
     NVIC_DisableIRQ(EXTI9_5_IRQn);
     NVIC_DisableIRQ(EXTI15_10_IRQn);
}

void exti_controller::register_callback(Input_pin* a_p_pin,
                                        Interrupt_mode a_mode,
                                        const Callback& a_callback)
{
    assert(nullptr != a_p_pin);
    assert(true == mcu::is_syscfg_enabled());
    assert(nullptr == handlers[static_cast<uint32_t>(a_p_pin->get_pin())].callback.function);

    set_flag(&(SYSCFG->EXTICR[a_p_pin->get_pin() / 4u]),
            (static_cast<uint32>(a_p_pin->get_port()->get_id()) << ((static_cast<uint32_t>(a_p_pin->get_pin()) % 4u) * 4u)));

    clear_bit(&(EXTI->RTSR1), a_p_pin->get_pin());
    clear_bit(&(EXTI->FTSR1), a_p_pin->get_pin());
    set_bit(&(EXTI->IMR1), a_p_pin->get_pin());

    switch (a_mode)
    {
        case Interrupt_mode::rising:
        {
            set_bit(&(EXTI->RTSR1), a_p_pin->get_pin());
        }
        break;

        case Interrupt_mode::falling:
        {
            set_bit(&(EXTI->FTSR1), a_p_pin->get_pin());
        }
        break;

        default:
        {
            if ((Interrupt_mode::rising | Interrupt_mode::falling) == a_mode)
            {
                set_bit(&(EXTI->RTSR1), a_p_pin->get_pin());
                set_bit(&(EXTI->FTSR1), a_p_pin->get_pin());
            }
        }
    }

    handlers[a_p_pin->get_pin()] = { a_callback, a_p_pin };
}

void exti_controller::unregister_callback(const Input_pin& a_pin)
{
    clear_bit(&(EXTI->RTSR1), a_pin.get_pin());
    clear_bit(&(EXTI->FTSR1), a_pin.get_pin());

    clear_flag(&(SYSCFG->EXTICR[a_pin.get_pin() / 4u]),
               (static_cast<uint32>(a_pin.get_port()->get_id()) << ((static_cast<uint32_t>(a_pin.get_pin()) % 4u) * 4u)));

    handlers[a_pin.get_pin()] = { { nullptr, nullptr }, nullptr };
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml