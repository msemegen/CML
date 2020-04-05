/*
    Name: I2C.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//this
#include <hal/stm32l452xx/I2C.hpp>

//cml
#include <hal/core/systick.hpp>
#include <utils/wait.hpp>

namespace {

using namespace cml::common;
using namespace cml::hal::stm32l452xx;

void i2c_1_enable(uint32 a_clock_source, uint32 a_irq_priority)
{
    set_flag(&(RCC->CCIPR), RCC_CCIPR_I2C1SEL, a_clock_source);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN);

    NVIC_SetPriority(I2C1_EV_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
}

void i2c_1_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C1EN);
    NVIC_DisableIRQ(I2C1_EV_IRQn);
}

void i2c_2_enable(uint32 a_clock_source, uint32 a_irq_priority)
{
    set_flag(&(RCC->CCIPR), RCC_CCIPR_I2C2SEL, a_clock_source);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C2EN);

    NVIC_SetPriority(I2C2_EV_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C2_EV_IRQn);
}

void i2c_2_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C2EN);
    NVIC_DisableIRQ(I2C2_EV_IRQn);
}

void i2c_3_enable(uint32 a_clock_source, uint32 a_irq_priority)
{
    set_flag(&(RCC->CCIPR), RCC_CCIPR_I2C3SEL, a_clock_source);
    set_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C3EN);

    NVIC_SetPriority(I2C3_EV_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C3_EV_IRQn);
}

void i2c_3_disable()
{
    clear_flag(&(RCC->APB1ENR1), RCC_APB1ENR1_I2C3EN);
    NVIC_DisableIRQ(I2C3_EV_IRQn);
}

void i2c_4_enable(uint32 a_clock_source, uint32 a_irq_priority)
{
    set_flag(&(RCC->CCIPR2), RCC_CCIPR2_I2C4SEL, a_clock_source);
    set_flag(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN);

    NVIC_SetPriority(I2C4_EV_IRQn, a_irq_priority);
    NVIC_EnableIRQ(I2C4_EV_IRQn);
}

void i2c_4_disable()
{
    clear_flag(&(RCC->APB1ENR2), RCC_APB1ENR2_I2C4EN);
    NVIC_DisableIRQ(I2C4_EV_IRQn);
}

uint32 get_clock_source_register_value(uint32 a_clock_source, uint32 a_i2c_id)
{
    if (a_i2c_id < 4)
    {
        return a_clock_source << (12 + a_i2c_id * 2);
    }
    else
    {
        return a_clock_source;
    }

    return 0;
}

struct Controller
{
    using Enable_function  = void(*)(uint32 a_clock_source, uint32 a_irq_priority);
    using Disable_function = void(*)();

    I2C_TypeDef* p_registers        = nullptr;
    I2C_master* p_i2c_master_handle = nullptr;
    I2C_slave* p_i2c_slave_handle   = nullptr;

    Enable_function enable   = nullptr;
    Disable_function disable = nullptr;
};

Controller controllers[]
{
    { I2C1, nullptr, nullptr, i2c_1_enable, i2c_1_disable },
    { I2C2, nullptr, nullptr, i2c_2_enable, i2c_2_disable },
    { I2C3, nullptr, nullptr, i2c_3_enable, i2c_3_disable },
    { I2C4, nullptr, nullptr, i2c_4_enable, i2c_4_disable }
};

} // namespace ::

extern "C"
{

void interupt_handler(uint32 a_controller_index)
{
    assert(nullptr != controllers[a_controller_index].p_i2c_master_handle ||
           nullptr != controllers[a_controller_index].p_i2c_slave_handle);

    if (nullptr != controllers[a_controller_index].p_i2c_master_handle)
    {
        i2c_handle_interrupt(controllers[a_controller_index].p_i2c_master_handle);
    }

    if (nullptr != controllers[a_controller_index].p_i2c_slave_handle)
    {
        i2c_handle_interrupt(controllers[a_controller_index].p_i2c_slave_handle);
    }
}

void I2C1_EV_IRQHandler()
{
    interupt_handler(0);
}

void I2C2_EV_IRQHandler()
{
    interupt_handler(1);
}

void I2C3_EV_IRQHandler()
{
    interupt_handler(2);
}

void I2C4_EV_IRQHandler()
{
    interupt_handler(3);
}

} // extern "C"

namespace cml {
namespace hal {
namespace stm32l452xx {

using namespace cml::common;
using namespace cml::hal::core;
using namespace cml::utils;

void i2c_handle_interrupt(I2C_master* a_p_this)
{

}

void i2c_handle_interrupt(I2C_slave* a_p_this)
{

}

void I2C_master::enable(Clock_source a_clock_source,
                        bool a_analog_filter,
                        uint32 a_timings,
                        bool a_is_fast_plus,
                        uint32 a_irq_priority)
{
    assert(nullptr == controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr == controllers[static_cast<uint32>(this->id)].p_i2c_slave_handle);

    controllers[static_cast<uint32>(this->id)].enable(get_clock_source_register_value(static_cast<uint32>(a_clock_source),
                                                                                      static_cast<uint32>(this->id)),
                                                      a_irq_priority);

    controllers[static_cast<uint32>(this->id)].p_i2c_master_handle = this;
    this->p_i2c = controllers[static_cast<uint32>(this->id)].p_registers;

    this->p_i2c->CR1     = 0;
    this->p_i2c->TIMINGR = a_timings;

    this->p_i2c->CR1 = (false == a_analog_filter ? I2C_CR1_ANFOFF : 0) | I2C_CR1_PE;

    if (true == a_is_fast_plus)
    {
        set_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32>(this->id));
    }
}

void I2C_master::diasble()
{
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);

    this->p_i2c->CR1 = 0;

    if (true == this->is_fast_plus())
    {
        clear_bit(&(SYSCFG->CFGR1), SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<uint32>(this->id));
    }

    controllers[static_cast<uint32>(this->id)].disable();
    controllers[static_cast<uint32>(this->id)].p_i2c_master_handle = nullptr;
    this->p_i2c = nullptr;
}

void I2C_master::write_bytes_polling(uint32 a_slave_address, const void* a_p_data, uint32 a_data_size_in_bytes)
{
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);

    uint32 address_mask = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_length_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_i2c->CR2 = address_mask | data_length_mask | I2C_CR2_AUTOEND | I2C_CR2_START;

    for (uint32 i = 0; i < a_data_size_in_bytes; i++)
    {
        wait::until(&(this->p_i2c->ISR), I2C_ISR_TXE, false);
        this->p_i2c->TXDR = static_cast<const uint8*>(a_p_data)[i];
    }

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    this->p_i2c->CR2 = 0;
}

bool I2C_master::write_bytes_polling(uint32 a_slave_address,
                                     const void* a_p_data,
                                     uint32 a_data_size_in_bytes,
                                     time_tick a_timeout_ms)
{
    assert(nullptr != controllers[static_cast<uint32>(this->id)].p_i2c_master_handle);
    assert(nullptr != a_p_data);
    assert(a_data_size_in_bytes > 0 && a_data_size_in_bytes <= 255);
    assert(true == systick::is_enabled());

    time_tick start = systick::get_counter();

    uint32 address_mask = (static_cast<uint32_t>(a_slave_address) << 1) & I2C_CR2_SADD;
    uint32 data_length_mask = static_cast<uint32_t>(a_data_size_in_bytes) << I2C_CR2_NBYTES_Pos;

    this->p_i2c->CR2 = address_mask | data_length_mask | I2C_CR2_AUTOEND | I2C_CR2_START;

    bool ret = true;
    for (uint32 i = 0; i < a_data_size_in_bytes && true == ret; i++)
    {
        ret = wait::until(&(this->p_i2c->ISR), I2C_ISR_TXE, false, start, a_timeout_ms);

        if (true == ret)
        {
            this->p_i2c->TXDR = static_cast<const uint8*>(a_p_data)[i];
        }
    }

    set_flag(&(this->p_i2c->ICR), I2C_ICR_STOPCF);
    this->p_i2c->CR2 = 0;

    return ret;
}

void I2C_master::read_bytes_polling(uint32 a_slave_address, void* a_p_data, uint32 a_data_size_in_bytes)
{
}

bool I2C_master::read_bytes_polling(uint32 a_slave_address,
                                    void* a_p_data,
                                    uint32 a_data_size_in_bytes,
                                    time_tick a_timeout_ms)
{
}

} // namespace stm32l452xx
} // namespace hal
} // namespace cml