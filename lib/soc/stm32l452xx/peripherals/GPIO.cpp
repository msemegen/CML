/*
 *   Name: GPIO.cpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#ifdef STM32L452xx

// this
#include <soc/stm32l452xx/peripherals/GPIO.hpp>

// soc
#include <soc/Interrupt_guard.hpp>

// cml
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#ifdef CML_ASSERT_ENABLED
#include <soc/stm32l452xx/mcu.hpp>
#endif //  CML_ASSERT

namespace {

using namespace cml;
using namespace soc::stm32l452xx::peripherals;

void gpio_a_enable()
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);
}

void gpio_a_disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);
}

void gpio_b_enable()
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);
}

void gpio_b_disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);
}

void gpio_c_enable()
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);
}

void gpio_c_disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);
}

void gpio_d_enable()
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);
}

void gpio_d_disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);
}

void gpio_e_enable()
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);
}

void gpio_e_disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);
}

void gpio_h_enable()
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);
}

void gpio_h_disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);
}

uint32_t get_index_from_Id(GPIO::Id a_id)
{
    return static_cast<uint32_t>(a_id);
}

struct Controller
{
    GPIO_TypeDef* p_registers = nullptr;

    void (*enable)()  = nullptr;
    void (*disable)() = nullptr;
};

Controller controllers[] = {
    { GPIOA, gpio_a_enable, gpio_a_disable },
    { GPIOB, gpio_b_enable, gpio_b_disable },
    { GPIOC, gpio_c_enable, gpio_c_disable },
    { GPIOD, gpio_d_enable, gpio_d_disable },
    { GPIOE, gpio_e_enable, gpio_e_disable },
    { nullptr, nullptr, nullptr },
    { nullptr, nullptr, nullptr },
    { GPIOH, gpio_h_enable, gpio_h_disable },
};

struct Interrupt_handler
{
    GPIO::EXTI::Callback callback;
};

Interrupt_handler interrupt_handlers[7];

} // namespace

extern "C" {

using namespace cml;
using namespace soc;

static bool interrupt_handler(uint32_t a_pr1, uint32_t a_index)
{
    if (true == bit::is(a_pr1, a_index))
    {
        interrupt_handlers[a_index].callback.function(a_index, interrupt_handlers[a_index].callback.p_user_data);
        return true;
    }

    return false;
}

#ifndef EXTI
#define EXTI ((EXTI_TypeDef*)EXTI_BASE)
#endif

void EXTI0_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 0))
    {
        bit::set(&(EXTI->PR1), 0);
    }
}

void EXTI1_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 1))
    {
        bit::set(&(EXTI->PR1), 1);
    }
}

void EXTI2_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 2))
    {
        bit::set(&(EXTI->PR1), 2);
    }
}

void EXTI3_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 3))
    {
        bit::set(&(EXTI->PR1), 3);
    }
}

void EXTI4_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 4))
    {
        bit::set(&(EXTI->PR1), 4);
    }
}

void EXTI9_5_IRQHandler()
{
    for (uint32_t i = 5u; i <= 9u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i))
        {
            bit::set(&(EXTI->PR1), i);
        }
    }
}

void EXTI15_10_IRQHandler()
{
    for (uint32_t i = 10u; i <= 15u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i))
        {
            bit::set(&(EXTI->PR1), i);
        }
    }
}

} // extern "C"

namespace soc {
namespace stm32l452xx {
namespace peripherals {

using namespace cml;

void GPIO::enable()
{
    this->p_gpio = controllers[get_index_from_Id(this->id)].p_registers;
    controllers[get_index_from_Id(this->id)].enable();

    bit::set(&(this->flags), 31u);
}

void GPIO::disable()
{
    controllers[get_index_from_Id(this->id)].disable();

    bit::clear(&(this->flags), 31);
}

void GPIO::In::Pin::set_pull(Pull a_pull)
{
    cml_assert(Pull::unknown != a_pull);
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
    cml_assert(Mode::unknown != a_mode);
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
                  0x1u << this->id,
                  static_cast<uint32_t>(a_mode) << this->id);
}

void GPIO::Out::Pin::set_pull(Pull a_pull)
{
    cml_assert(Pull::unknown != a_pull);
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<uint32_t>(a_pull) << (this->id * 2));
}

void GPIO::Out::Pin::set_speed(Speed a_speed)
{
    cml_assert(Speed::unknown != a_speed);
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
    cml_assert(Pull::unknown != a_pull);
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
    cml_assert(Mode::unknown != a_mode);
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
                  0x1u << this->id,
                  static_cast<uint32_t>(a_mode) << this->id);
}

void GPIO::Alternate_function::Pin::set_pull(Pull a_pull)
{
    cml_assert(Pull::unknown != a_pull);
    cml_assert(nullptr != this->p_port && 0xFF != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<uint32_t>(a_pull) << (this->id * 2));
}

void GPIO::Alternate_function::Pin::set_speed(Speed a_speed)
{
    cml_assert(Speed::unknown != a_speed);
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
    cml_assert(Pull::unknown != a_pull);

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

    cml_assert(Pull::unknown != a_config.pull);
    cml_assert(Speed::unknown != a_config.speed);
    cml_assert(Mode::unknown != a_config.mode);

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

    cml_assert(Pull::unknown != a_pull);

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

    cml_assert(Pull::unknown != a_config.pull);
    cml_assert(Speed::unknown != a_config.speed);
    cml_assert(Mode::unknown != a_config.mode);

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

#ifdef EXTI
#undef EXTI
#endif

void GPIO::EXTI::enable(const Callback& a_callback, uint32_t a_priority)
{
    cml_assert(true == mcu::is_syscfg_enabled());

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

void GPIO::EXTI::disable()
{
    NVIC_DisableIRQ(static_cast<IRQn_Type>(this->id));
}

#ifndef EXTI
#define EXTI ((EXTI_TypeDef*)EXTI_BASE)
#endif

#ifdef EXTI
#undef EXTI
#endif

void GPIO::EXTI::attach(const GPIO& a_port, uint32_t a_pin, Trigger_flag a_trigger)
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
#ifndef EXTI
#define EXTI ((EXTI_TypeDef*)EXTI_BASE)
#endif

    Interrupt_guard guard;

    bit_flag::set(p_register, 0x3u << pos, static_cast<uint32_t>(a_port.get_id()) << pos);

    bit::clear(&(EXTI->RTSR1), a_pin);
    bit::clear(&(EXTI->FTSR1), a_pin);
    bit::set(&(EXTI->IMR1), a_pin);

    switch (a_trigger)
    {
        case Trigger_flag::rising: {
            bit::set(&(EXTI->RTSR1), a_pin);
        }
        break;

        case Trigger_flag::falling: {
            bit::set(&(EXTI->FTSR1), a_pin);
        }
        break;

        default: {
            if ((Trigger_flag::rising | Trigger_flag::falling) == a_trigger)
            {
                bit::set(&(EXTI->RTSR1), a_pin);
                bit::set(&(EXTI->FTSR1), a_pin);
            }
        }
    }
}

#ifdef EXTI
#undef EXTI
#endif

void GPIO::EXTI::deattach(const GPIO& a_port, uint32_t a_pin)
{
    Interrupt_guard guard;
#ifndef EXTI
#define EXTI ((EXTI_TypeDef*)EXTI_BASE)
#endif

    bit::clear(&(EXTI->RTSR1), a_pin);
    bit::clear(&(EXTI->FTSR1), a_pin);

    bit_flag::clear(&(SYSCFG->EXTICR[a_pin / 4u]),
                    (static_cast<uint32_t>(a_port.get_id()) << ((static_cast<uint32_t>(a_pin) % 4u) * 4u)));

    interrupt_handlers[a_pin].callback = { nullptr, nullptr };
}

} // namespace peripherals
} // namespace stm32l452xx
} // namespace soc

#endif // STM32L452xx