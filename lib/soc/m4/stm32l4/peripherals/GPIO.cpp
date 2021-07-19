/*
 *   Name: GPIO.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L4

// this
#include <soc/m4/stm32l4/peripherals/GPIO.hpp>

// soc
#include <soc/Interrupt_guard.hpp>
#include <soc/m4/stm32l4/mcu.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>

namespace {

using namespace cml;
using namespace soc::m4::stm32l4;
using namespace soc::m4::stm32l4::peripherals;

uint32_t get_index_from_Id(GPIO::Id a_id)
{
    return static_cast<uint32_t>(a_id);
}

struct Controller
{
    GPIO_TypeDef* p_registers = nullptr;
};

struct Interrupt_handler
{
    EXTI<GPIO>::Callback callback;
};

Interrupt_handler interrupt_handlers[7];

Controller controllers[] = {
    { GPIOA },   { GPIOB },   { GPIOC },
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
    { GPIOD },
#else
    { nullptr },
#endif
#if defined(STM32L431xx) || defined(STM32L433xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)
    { GPIOE },
#else
    { nullptr },
#endif
    { nullptr }, { nullptr }, { GPIOH },
};

} // namespace

extern "C" {

using namespace cml;

#define EXTI_T ((EXTI_TypeDef*)EXTI_BASE)

static bool interrupt_handler(uint32_t a_pr1, uint32_t a_bit_index, uint32_t a_handler_index)
{
    if (true == bit::is(a_pr1, a_bit_index))
    {
        interrupt_handlers[a_handler_index].callback.function(a_bit_index,
                                                              interrupt_handlers[a_handler_index].callback.p_user_data);
        return true;
    }

    return false;
}

void EXTI0_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 0, 0))
    {
        bit::set(&(EXTI_T->PR1), 0);
    }
}

void EXTI1_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 1, 1))
    {
        bit::set(&(EXTI_T->PR1), 1);
    }
}

void EXTI2_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 2, 2))
    {
        bit::set(&(EXTI_T->PR1), 2);
    }
}

void EXTI3_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 3, 3))
    {
        bit::set(&(EXTI_T->PR1), 3);
    }
}

void EXTI4_IRQHandler()
{
    if (true == interrupt_handler(EXTI_T->PR1, 4, 4))
    {
        bit::set(&(EXTI_T->PR1), 4);
    }
}

void EXTI9_5_IRQHandler()
{
    for (uint32_t i = 5u; i <= 9u; i++)
    {
        if (true == interrupt_handler(EXTI_T->PR1, i, 5))
        {
            bit::set(&(EXTI_T->PR1), i);
        }
    }
}

void EXTI15_10_IRQHandler()
{
    for (uint32_t i = 10u; i <= 15u; i++)
    {
        if (true == interrupt_handler(EXTI_T->PR1, i, 6))
        {
            bit::set(&(EXTI_T->PR1), i);
        }
    }
}
}

namespace soc {
namespace m4 {
namespace stm32l4 {
namespace peripherals {

using namespace cml;

void GPIO::enable()
{
    this->p_gpio = controllers[get_index_from_Id(this->id)].p_registers;
    bit::set(&(this->flags), 31u);
}

void GPIO::disable()
{
    bit::clear(&(this->flags), 31);
}

void GPIO::In::Pin::set_pull(Pull a_pull)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<uint32_t>(a_pull) << (this->id * 2));
}

GPIO::Level GPIO::In::Pin::get_level() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Level>(bit::is(static_cast<GPIO_TypeDef*>(*(this->p_port))->IDR, this->id));
}

GPIO::Pull GPIO::In::Pin::get_pull() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(
        (bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR, static_cast<uint32_t>(0x3u << this->id))
         << this->id));
}

void GPIO::Out::Pin::set_level(Level a_level)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    constexpr uint8_t mul[]                           = { 16, 0 };
    static_cast<GPIO_TypeDef*>(*(this->p_port))->BSRR = 0x1u << (this->id + mul[static_cast<uint32_t>(a_level)]);
}

void GPIO::Out::Pin::toggle_level()
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit::toggle(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->ODR), this->id);
}

void GPIO::Out::Pin::set_mode(Mode a_mode)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
                  0x1u << this->id,
                  static_cast<uint32_t>(a_mode) << this->id);
}

void GPIO::Out::Pin::set_pull(Pull a_pull)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<uint32_t>(a_pull) << (this->id * 2));
}

void GPIO::Out::Pin::set_speed(Speed a_speed)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR),
                  0x3u << (this->id * 2),
                  static_cast<uint32_t>(a_speed) << (this->id * 2));
}

GPIO::Level GPIO::Out::Pin::get_level() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Level>(bit::is(static_cast<GPIO_TypeDef*>(*(this->p_port))->IDR, this->id));
}

GPIO::Mode GPIO::Out::Pin::get_mode() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Mode>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER,
                                           static_cast<uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Pull GPIO::Out::Pin::get_pull() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                           static_cast<uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Speed GPIO::Out::Pin::get_speed() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Speed>((bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR,
                                             static_cast<uint32_t>(0x1u << this->id) << this->id)));
}

void GPIO::Analog::Pin::set_pull(Pull a_pull)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<uint32_t>(a_pull) << (this->id * 2));
}

GPIO::Pull GPIO::Analog::Pin::get_pull() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                           static_cast<uint32_t>(0x1u << this->id) << this->id));
}

void GPIO::Alternate_function::Pin::set_mode(Mode a_mode)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
                  0x1u << this->id,
                  static_cast<uint32_t>(a_mode) << this->id);
}

void GPIO::Alternate_function::Pin::set_pull(Pull a_pull)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<uint32_t>(a_pull) << (this->id * 2));
}

void GPIO::Alternate_function::Pin::set_speed(Speed a_speed)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR),
                  0x3u << (this->id * 2),
                  static_cast<uint32_t>(a_speed) << (this->id * 2));
}

void GPIO::Alternate_function::Pin::set_function(uint32_t a_function)
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    uint32_t af_register_index = this->id >> 3u;
    uint32_t af_register       = p_port->AFR[af_register_index];

    af_register &= ~(0xFu << ((this->id - (af_register_index * 8u)) * 4u));
    af_register |= a_function << ((this->id - (af_register_index * 8u)) * 4u);

    p_port->AFR[af_register_index] = af_register;
    this->function                 = a_function;
}

GPIO::Mode GPIO::Alternate_function::Pin::get_mode() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Mode>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER,
                                           static_cast<uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Pull GPIO::Alternate_function::Pin::get_pull() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                           static_cast<uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Speed GPIO::Alternate_function::Pin::get_speed() const
{
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Speed>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR,
                                            static_cast<uint32_t>(0x1u << this->id) << this->id));
}

void GPIO::In::enable(uint32_t a_id, Pull a_pull, Pin* a_p_pin)
{
    cml_assert(a_id < 16);

    cml_assert(true == this->p_port->is_enabled());
    cml_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*(this->p_port)));

    bit_flag::set(&(p_port->PUPDR), 0x3u << (a_id * 2), static_cast<uint32_t>(a_pull) << (a_id * 2));
    bit_flag::clear(&(p_port->MODER), 0x3u << (a_id * 2));

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id     = a_id;
        a_p_pin->p_port = this->p_port;
    }
}

void GPIO::In::disable(uint32_t a_id)
{
    cml_assert(a_id < 16);

    cml_assert(true == this->p_port->is_enabled());
    cml_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    const uint32_t flag = (0x3u << (a_id * 2));

    bit_flag::set(&(p_port->MODER), flag);
    bit_flag::clear(&(p_port->PUPDR), flag);

    this->p_port->give_pin(a_id);
}

void GPIO::Out::enable(uint32_t a_id, const Config& a_config, Pin* a_p_pin)
{
    cml_assert(a_id < 16);

    cml_assert(various::get_enum_incorrect_value<Pull>() != a_config.pull);
    cml_assert(various::get_enum_incorrect_value<Speed>() != a_config.speed);
    cml_assert(various::get_enum_incorrect_value<Mode>() != a_config.mode);

    cml_assert(true == this->p_port->is_enabled());
    cml_assert(false == this->p_port->is_pin_taken(a_id));

    const uint32_t clear_flag_2bit = 0x3u << (a_id * 2);
    GPIO_TypeDef* p_port           = static_cast<GPIO_TypeDef*>(*(this->p_port));

    bit_flag::set(&(p_port->OSPEEDR), clear_flag_2bit, static_cast<uint32_t>(a_config.speed) << (a_id * 2));
    bit_flag::set(&(p_port->PUPDR), clear_flag_2bit, static_cast<uint32_t>(a_config.pull) << (a_id * 2));
    bit_flag::set(&(p_port->MODER), clear_flag_2bit, 0x1u << (a_id * 2));
    bit_flag::set(&(p_port->OTYPER), 0x1u << a_id, static_cast<uint32_t>(a_config.mode) << a_id);

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id     = a_id;
        a_p_pin->p_port = this->p_port;
    }
}

void GPIO::Out::disable(uint32_t a_id)
{
    cml_assert(a_id < 16);

    cml_assert(true == this->p_port->is_enabled());
    cml_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    const uint32_t flag = (0x3u << (a_id * 2));

    bit_flag::set(&(p_port->MODER), flag);
    bit_flag::clear(&(p_port->OSPEEDR), flag);
    bit_flag::clear(&(p_port->PUPDR), flag);

    this->p_port->give_pin(a_id);
}

void GPIO::Analog::enable(uint32_t a_id, Pull a_pull, Pin* a_p_out_pin)
{
    cml_assert(a_id < 16);
    cml_assert(true == this->p_port->is_enabled());
    cml_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    bit_flag::set(&(p_port->PUPDR), 0x3u << (a_id * 2), static_cast<uint32_t>(a_pull) << (a_id * 2));
    bit_flag::set(&(p_port->MODER), 0x3u << (a_id * 2), 0x3u << (a_id * 2));

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_out_pin)
    {
        a_p_out_pin->id     = a_id;
        a_p_out_pin->p_port = this->p_port;
    }
}

void GPIO::Analog::disable(uint32_t a_id)
{
    cml_assert(a_id < 16);
    cml_assert(true == this->p_port->is_enabled());

    bit_flag::clear(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR), (0x3u << (a_id * 2)));

    this->p_port->give_pin(a_id);
}

void GPIO::Alternate_function::enable(uint32_t a_id, const Config& a_config, Pin* a_p_pin)
{
    cml_assert(a_id < 16);

    cml_assert(true == this->p_port->is_enabled());
    cml_assert(false == this->p_port->is_pin_taken(a_id));

    cml_assert(various::get_enum_incorrect_value<Pull>() != a_config.pull);
    cml_assert(various::get_enum_incorrect_value<Speed>() != a_config.speed);
    cml_assert(various::get_enum_incorrect_value<Mode>() != a_config.mode);

    const uint32_t clear_flag_2bit = 0x3u << (a_id * 2);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    bit_flag::set(&(p_port->OSPEEDR), clear_flag_2bit, static_cast<uint32_t>(a_config.speed) << (a_id * 2));
    bit_flag::set(&(p_port->PUPDR), clear_flag_2bit, static_cast<uint32_t>(a_config.pull) << (a_id * 2));
    bit_flag::set(&(p_port->MODER), clear_flag_2bit, 0x2u << (a_id * 2));
    bit_flag::set(&(p_port->OTYPER), 0x1u << a_id, static_cast<uint32_t>(a_config.mode) << a_id);

    uint32_t af_register_index = a_id >> 3u;
    uint32_t af_register       = p_port->AFR[af_register_index];

    af_register &= ~(0xFu << ((a_id - (af_register_index * 8u)) * 4u));
    af_register |= a_config.function << ((a_id - (af_register_index * 8u)) * 4u);

    p_port->AFR[af_register_index] = af_register;

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id       = a_id;
        a_p_pin->p_port   = this->p_port;
        a_p_pin->function = a_config.function;
    }
}

void GPIO::Alternate_function::disable(uint32_t a_id)
{
    cml_assert(a_id < 16);
    cml_assert(true == this->p_port->is_enabled());

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    const uint32_t flag = (0x3u << (a_id * 2));

    bit_flag::set(&(p_port->MODER), flag);
    bit_flag::clear(&(p_port->OSPEEDR), flag);
    bit_flag::clear(&(p_port->PUPDR), flag);

    this->p_port->give_pin(a_id);
}

} // namespace peripherals
} // namespace stm32l4
} // namespace m4
} // namespace soc

namespace soc {
namespace m4 {
namespace stm32l4 {

using namespace soc::m4::stm32l4::peripherals;

void rcc<GPIO>::enable(GPIO::Id a_id, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), 0x1ul << (RCC_AHB2ENR_GPIOAEN_Pos + static_cast<uint32_t>(a_id)));

    if (true == a_enable_in_lp)
    {
        bit::set(&(RCC->AHB2SMENR), static_cast<uint32_t>(a_id));
    }
}

void rcc<GPIO>::disable(GPIO::Id a_id)
{
    bit_flag::clear(&(RCC->AHB2ENR), 0x1ul << (RCC_AHB2ENR_GPIOAEN_Pos + static_cast<uint32_t>(a_id)));
    bit::clear(&(RCC->AHB2SMENR), static_cast<uint32_t>(a_id));
}

void EXTI<GPIO>::enable(const Callback& a_callback, uint32_t a_priority)
{
    cml_assert(rcc<mcu>::SYSCFG_mode::enabled == rcc<mcu>::get_SYSCFG_mode());

    NVIC_SetPriority(static_cast<IRQn_Type>(this->id), a_priority);
    NVIC_EnableIRQ(static_cast<IRQn_Type>(this->id));

    if (this->id >= Id::_0 && this->id <= Id::_4)
    {
        interrupt_handlers[static_cast<uint32_t>(this->id) - static_cast<uint32_t>(Id::_0)].callback = a_callback;
    }
    else
    {
        if (Id::_5_9 == this->id)
        {
            interrupt_handlers[5].callback = a_callback;
        }
        else if (Id::_10_15 == this->id)
        {
            interrupt_handlers[6].callback = a_callback;
        }
    }
}

void EXTI<GPIO>::disable()
{
    NVIC_DisableIRQ(static_cast<IRQn_Type>(this->id));
}

void EXTI<GPIO>::attach(const GPIO& a_port, uint32_t a_pin, Trigger_flag a_trigger, Mode a_mode)
{
    volatile uint32_t* p_register = &(SYSCFG->EXTICR[a_pin / 4u]);
    uint32_t pos                  = ((static_cast<uint32_t>(a_pin) % 4u) * 4u);

#ifdef CML_ASSERT_ENABLED
    const bool f = bit_flag::is(*p_register, static_cast<uint32_t>(a_port.get_id()) << pos);
    cml_assert((GPIO::Id::a == a_port.get_id() && true == f) || (GPIO::Id::a != a_port.get_id() && false == f));
    cml_assert((Id::_0 == this->id && a_pin == 0u) || (Id::_1 == this->id && a_pin == 1u) ||
               (Id::_2 == this->id && a_pin == 2u) || (Id::_3 == this->id && a_pin == 3u) ||
               (Id::_4 == this->id && a_pin == 4u) || (Id::_5_9 == this->id && (a_pin >= 5u && a_pin <= 9u)) ||
               (Id::_10_15 == this->id && (a_pin >= 10u && a_pin <= 15u)));
#endif

    Interrupt_guard guard;

    bit_flag::set(p_register, 0x3u << pos, static_cast<uint32_t>(a_port.get_id()) << pos);

    bit::clear(&(EXTI_T->RTSR1), a_pin);
    bit::clear(&(EXTI_T->FTSR1), a_pin);

    switch (a_mode)
    {
        case Mode::event: {
            bit::set(&(EXTI_T->EMR1), a_pin);
        }
        break;

        case Mode::interrupt: {
            bit::set(&(EXTI_T->IMR1), a_pin);
        }
        break;
    }

    switch (a_trigger)
    {
        case Trigger_flag::rising: {
            bit::set(&(EXTI_T->RTSR1), a_pin);
        }
        break;

        case Trigger_flag::falling: {
            bit::set(&(EXTI_T->FTSR1), a_pin);
        }
        break;

        default: {
            if ((Trigger_flag::rising | Trigger_flag::falling) == a_trigger)
            {
                bit::set(&(EXTI_T->RTSR1), a_pin);
                bit::set(&(EXTI_T->FTSR1), a_pin);
            }
        }
    }
}

void EXTI<GPIO>::deattach(const GPIO& a_port, uint32_t a_pin)
{
    Interrupt_guard guard;

    bit::clear(&(EXTI_T->RTSR1), a_pin);
    bit::clear(&(EXTI_T->FTSR1), a_pin);

    bit::clear(&(EXTI_T->EMR1), a_pin);
    bit::clear(&(EXTI_T->IMR1), a_pin);

    bit_flag::clear(&(SYSCFG->EXTICR[a_pin / 4u]),
                    (static_cast<uint32_t>(a_port.get_id()) << ((static_cast<uint32_t>(a_pin) % 4u) * 4u)));

    interrupt_handlers[a_pin].callback = { nullptr, nullptr };
}


} // namespace stm32l4
} // namespace m4
} // namespace soc

#endif // STM32L4