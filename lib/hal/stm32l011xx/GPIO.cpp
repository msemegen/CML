/*
    Name: GPIO.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

#ifdef STM32L011xx

//this
#include <hal/stm32l011xx/gpio.hpp>

namespace
{

using namespace cml::common;

void gpio_a_enable()
{
    set_flag(&(RCC->IOPENR), RCC_IOPENR_GPIOAEN);
}

void gpio_a_disable()
{
    clear_flag(&(RCC->IOPENR), RCC_IOPENR_GPIOAEN);
}

void gpio_b_enable()
{
    set_flag(&(RCC->IOPENR), RCC_IOPENR_GPIOBEN);
}

void gpio_b_disable()
{
    clear_flag(&(RCC->IOPENR), RCC_IOPENR_GPIOBEN);
}

void gpio_c_enable()
{
    set_flag(&(RCC->IOPENR), RCC_IOPENR_GPIOCEN);
}

void gpio_c_disable()
{
    clear_flag(&(RCC->IOPENR), RCC_IOPENR_GPIOCEN);
}

struct Controller
{
    GPIO_TypeDef* p_registers = nullptr;

    void(*p_enable)()  = nullptr;
    void(*p_disable)() = nullptr;
};

Controller controllers[] =
{
    { GPIOA, gpio_a_enable, gpio_a_disable },
    { GPIOB, gpio_b_enable, gpio_b_disable },
    { GPIOC, gpio_c_enable, gpio_c_disable },
};

} // namespace ::

namespace cml {
namespace hal {
namespace stm32l011xx {

using namespace cml::common;

void GPIO::enable()
{
    this->p_gpio = controllers[this->to_index(this->id)].p_registers;
    controllers[this->to_index(this->id)].p_enable();

    set_bit(&(this->flags), 31u);
}

void GPIO::disable()
{
    controllers[this->to_index(this->id)].p_disable();

    common::clear_bit(&(this->flags), 31);
}

void Output_pin::enable(const Config& a_config)
{
    assert(false == this->p_port->is_pin_taken(this->pin));

    assert(Pull::unknown  != a_config.pull);
    assert(Speed::unknown != a_config.speed);
    assert(Mode::unknown  != a_config.mode);

    this->set_speed(a_config.speed);
    this->set_pull(a_config.pull);
    this->set_mode(a_config.mode);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->MODER), 0x3u << (this->pin * 2), 0x1u << (this->pin * 2));

    this->p_port->take_pin(this->pin);
}

void Output_pin::disable()
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    uint32 flag = (0x3u << (this->pin * 2));

    set_flag(&(p_port->MODER),     flag);
    clear_flag(&(p_port->OSPEEDR), flag);
    clear_flag(&(p_port->PUPDR),   flag);

    this->p_port->give_pin(this->pin);
}

void Output_pin::set_level(Level a_level)
{
    uint8_t mul[] = { 16, 0 };
    static_cast<GPIO_TypeDef*>((*this->p_port))->BSRR = 0x1u << (this->pin + mul[static_cast<uint32>(a_level)]);
}

void Output_pin::toggle_level()
{
    toggle_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->ODR), this->pin);
}

void Output_pin::set_mode(Mode a_mode)
{
    switch (a_mode)
    {
        case Mode::open_drain:
        {
            set_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->OTYPER), this->pin);
        }
        break;

        case Mode::push_pull:
        {
            clear_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->OTYPER), this->pin);
        }
        break;

        default:
        {
            assert(Mode::unknown != a_mode);
        }
    }
}

void Output_pin::set_pull(Pull a_pull)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->PUPDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_pull) << (this->pin * 2));
}

void Output_pin::set_speed(Speed a_speed)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->OSPEEDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_speed) << (this->pin * 2));
}

Output_pin::Mode Output_pin::get_mode() const
{
    const GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    return static_cast<Mode>(get_flag(p_port->OTYPER, static_cast<uint32>(0x1u << this->pin) << this->pin));
}

Output_pin::Pull Output_pin::get_pull() const
{
    const GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    return static_cast<Pull>(get_flag(p_port->PUPDR, static_cast<uint32>(0x1u << this->pin) << this->pin));
}

Output_pin::Speed Output_pin::get_speed() const
{
    const GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    return static_cast<Speed>(get_flag(p_port->OSPEEDR, static_cast<uint32>(0x1u << this->pin) << this->pin));
}

void Input_pin::enable(const Config& a_config)
{
    assert(false == this->p_port->is_pin_taken(this->pin));
    assert(Pull::unknown != a_config.pull);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    clear_flag(&(p_port->MODER), 0x3u << (this->pin * 2));

    this->set_pull(a_config.pull);
    this->p_port->take_pin(this->pin);
}

void Input_pin::disable()
{
    const uint32 flag = (0x3u << (this->pin * 2));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    set_flag(&(p_port->MODER),     flag);
    clear_flag(&(p_port->PUPDR),   flag);

    this->p_port->give_pin(this->pin);
}

void Input_pin::set_pull(Pull a_pull)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->PUPDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_pull) << (this->pin * 2));
}

Input_pin::Pull Input_pin::get_pull() const
{
    const GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    return static_cast<Pull>(get_flag(p_port->PUPDR, static_cast<uint32>(0x3u << this->pin) << this->pin));
}

Input_pin::Level Input_pin::get_level() const
{
    return static_cast<Level>(get_bit(static_cast<GPIO_TypeDef*>((*this->p_port))->IDR, this->pin));
}

void Alternate_function_pin::enable(const Config& a_config)
{
    assert(false == this->p_port->is_pin_taken(this->pin));

    assert(0 != a_config.function);
    assert(Pull::unknown  != a_config.pull);
    assert(Speed::unknown != a_config.speed);
    assert(Mode::unknown  != a_config.mode);

    this->set_speed(a_config.speed);
    this->set_pull(a_config.pull);
    this->set_mode(a_config.mode);
    this->set_function(a_config.function);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->MODER), 0x3u << (this->pin * 2), 0x2u << (this->pin * 2));

    this->p_port->take_pin(this->pin);
}

void Alternate_function_pin::disable()
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    const uint32 flag = (0x3u << (this->pin * 2));

    set_flag(&(p_port->MODER),     flag);
    clear_flag(&(p_port->OSPEEDR), flag);
    clear_flag(&(p_port->PUPDR),   flag);

    this->p_port->give_pin(this->pin);
}

void Alternate_function_pin::set_mode(Mode a_mode)
{
    switch (a_mode)
    {
        case Mode::open_drain:
        {
            set_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->OTYPER), this->pin);
        }
        break;

        case Mode::push_pull:
        {
            clear_bit(&(static_cast<GPIO_TypeDef*>((*this->p_port))->OTYPER), this->pin);
        }
        break;

        default:
        {
            assert(Mode::unknown != a_mode);
        }
    }
}

void Alternate_function_pin::set_pull(Pull a_pull)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->PUPDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_pull) << (this->pin * 2));
}

void Alternate_function_pin::set_speed(Speed a_speed)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));
    set_flag(&(p_port->OSPEEDR), 0x3u << (this->pin * 2), static_cast<uint32>(a_speed) << (this->pin * 2));
}

void Alternate_function_pin::set_function(uint32 a_function)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*this->p_port));

    uint32 af_register_index = this->pin >> 3u;
    uint32 af_register       = p_port->AFR[af_register_index];

    af_register &= ~(0xFu << ((this->pin - (af_register_index * 8u)) * 4u));
    af_register |= a_function << ((this->pin - (af_register_index * 8u)) * 4u);

    p_port->AFR[af_register_index] = af_register;
}

} // namespace stm32l011xx
} // namespace hal
} // namespace cml

#endif // STM32L011xx