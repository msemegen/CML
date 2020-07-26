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
    pin::In const* p_pin = nullptr;
};

Handler handlers[16];

} // namespace ::

extern "C"
{

using namespace cml;

static bool interrupt_handler(uint32_t a_pr1, uint32_t a_index)
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
    for (uint32_t i = 5u; i <= 9u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i))
        {
            set_bit(&(EXTI->PR1), i);
        }
    }
}

void EXTI15_10_IRQHandler()
{
    for (uint32_t i = 10u; i <= 15u; i++)
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

void exti_controller::enable(uint32_t a_priority)
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

void exti_controller::register_callback(pin::In* a_p_pin,
                                        Interrupt_mode a_mode,
                                        const Callback& a_callback)
{
    assert(nullptr != a_p_pin);
    assert(true == mcu::is_syscfg_enabled());
    assert(nullptr == handlers[static_cast<uint32_t>(a_p_pin->get_id())].callback.function);

    set_flag(&(SYSCFG->EXTICR[a_p_pin->get_id() / 4u]),
            (static_cast<uint32_t>(a_p_pin->get_port()->get_id()) << ((static_cast<uint32_t>(a_p_pin->get_id()) % 4u) * 4u)));

    clear_bit(&(EXTI->RTSR1), a_p_pin->get_id());
    clear_bit(&(EXTI->FTSR1), a_p_pin->get_id());
    set_bit(&(EXTI->IMR1), a_p_pin->get_id());

    switch (a_mode)
    {
        case Interrupt_mode::rising:
        {
            set_bit(&(EXTI->RTSR1), a_p_pin->get_id());
        }
        break;

        case Interrupt_mode::falling:
        {
            set_bit(&(EXTI->FTSR1), a_p_pin->get_id());
        }
        break;

        default:
        {
            if ((Interrupt_mode::rising | Interrupt_mode::falling) == a_mode)
            {
                set_bit(&(EXTI->RTSR1), a_p_pin->get_id());
                set_bit(&(EXTI->FTSR1), a_p_pin->get_id());
            }
        }
    }

    handlers[a_p_pin->get_id()] = { a_callback, a_p_pin };
}

void exti_controller::unregister_callback(const pin::In& a_pin)
{
    clear_bit(&(EXTI->RTSR1), a_pin.get_id());
    clear_bit(&(EXTI->FTSR1), a_pin.get_id());

    clear_flag(&(SYSCFG->EXTICR[a_pin.get_id() / 4u]),
               (static_cast<uint32_t>(a_pin.get_port()->get_id()) << ((static_cast<uint32_t>(a_pin.get_id()) % 4u) * 4u)));

    handlers[a_pin.get_id()] = { { nullptr, nullptr }, nullptr };
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml