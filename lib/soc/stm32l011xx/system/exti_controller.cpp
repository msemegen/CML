/*
    Name: exti_controller.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

//this
#include <soc/stm32l011xx/system/exti_controller.hpp>

//soc
#ifdef CML_ASSERT
#include <soc/stm32l011xx/mcu.hpp>
#endif

//cml
#include <cml/debug/assert.hpp>

namespace {

using namespace soc::stm32l011xx::peripherals;
using namespace soc::stm32l011xx::system;

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

static bool interrupt_handler(uint32 a_pr1, uint32 a_index)
{
    assert(nullptr != handlers[a_index].callback.function);

    if (true == is_bit(a_pr1, a_index))
    {
        return handlers[a_index].callback.function(handlers[a_index].p_pin->get_level(),
                                                   handlers[a_index].callback.p_user_data);

    }

    return false;
}

void EXTI0_1_IRQHandler()
{
    for (uint32 i = 0u; i <= 1u; i++)
    {
        if (true == interrupt_handler(EXTI->PR, i))
        {
            set_bit(&(EXTI->PR), i);
        }
    }
}

void EXTI2_3_IRQHandler()
{
    for (uint32 i = 2u; i <= 3u; i++)
    {
        if (true == interrupt_handler(EXTI->PR, i))
        {
            set_bit(&(EXTI->PR), i);
        }
    }
}

void EXTI4_15_IRQHandler()
{
    for (uint32 i = 4u; i <= 15u; i++)
    {
        if (true == interrupt_handler(EXTI->PR, i))
        {
            set_bit(&(EXTI->PR), i);
        }
    }
}

} // extern "C"

namespace soc {
namespace stm32l011xx {
namespace system {

using namespace cml;

void exti_controller::enable(uint32 a_priority)
{
    NVIC_SetPriority(EXTI0_1_IRQn,  a_priority);
    NVIC_SetPriority(EXTI2_3_IRQn,  a_priority);
    NVIC_SetPriority(EXTI4_15_IRQn, a_priority);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void exti_controller::disable()
{
    NVIC_DisableIRQ(EXTI0_1_IRQn);
    NVIC_DisableIRQ(EXTI2_3_IRQn);
    NVIC_DisableIRQ(EXTI4_15_IRQn);
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

    clear_bit(&(EXTI->RTSR), a_p_pin->get_pin());
    clear_bit(&(EXTI->FTSR), a_p_pin->get_pin());
    set_bit(&(EXTI->IMR), a_p_pin->get_pin());

    switch (a_mode)
    {
        case Interrupt_mode::rising:
        {
            set_bit(&(EXTI->RTSR), a_p_pin->get_pin());
        }
        break;

        case Interrupt_mode::falling:
        {
            set_bit(&(EXTI->FTSR), a_p_pin->get_pin());
        }
        break;

        default:
        {
            if ((Interrupt_mode::rising | Interrupt_mode::falling) == a_mode)
            {
                set_bit(&(EXTI->RTSR), a_p_pin->get_pin());
                set_bit(&(EXTI->FTSR), a_p_pin->get_pin());
            }
        }
    }

    handlers[a_p_pin->get_pin()] = { a_callback, a_p_pin };
}

void exti_controller::unregister_callback(const Input_pin& a_pin)
{
    clear_bit(&(EXTI->RTSR), a_pin.get_pin());
    clear_bit(&(EXTI->FTSR), a_pin.get_pin());

    clear_flag(&(SYSCFG->EXTICR[a_pin.get_pin() / 4u]),
               (static_cast<uint32>(a_pin.get_port()->get_id()) << ((static_cast<uint32_t>(a_pin.get_pin()) % 4u) * 4u)));

    handlers[a_pin.get_pin()] = { { nullptr, nullptr }, nullptr };
}

} // namespace system
} // namespace stm32l011xx
} // namespace soc

#endif // STM32L011xx