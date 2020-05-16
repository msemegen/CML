#pragma once

/*
    Name: I2C.hpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l4xx.h>

//cml
#include <common/bit.hpp>
#include <common/integer.hpp>
#include <common/Non_copyable.hpp>
#include <common/time_tick.hpp>
#include <debug/assert.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class I2C_master : private common::Non_copyable
{
public:

    enum class Id : common::uint32
    {
        _1,
        _2,
        _3,
        _4
    };

    enum class Clock_source
    {
        pclk1  = 0,
        sysclk = 1,
        hsi    = 2
    };

    enum class Bus_status
    {
        ok,
        timeout,
        overrun,
        arbitration_lost,
        misplaced,
        nack
    };

    struct Config
    {
        bool analog_filter     = false;
        common::uint32 timings = 0;
        bool fast_plus         = false;
    };

    struct TX_callback
    {
        using Function = bool(*)(volatile common::uint32* p_a_data, Bus_status a_status, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct RX_callback
    {
        using Function = bool(*)(common::uint8 a_data, Bus_status a_status, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:

    I2C_master(Id a_id)
        : id(a_id)
        , p_i2c(nullptr)
    {}

    ~I2C_master()
    {
        this->diasble();
    }

    void enable(const Config& a_config,
                Clock_source a_clock_source,
                common::uint32 a_irq_priority);
    void diasble();

    template<typename Data_t>
    common::uint32 transmit_polling(common::uint16 a_slave_address,
                                    const Data_t& a_data,
                                    Bus_status* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(a_slave_address, static_cast<void*>(a_data), sizeof(a_data), a_p_status);
    }

    template<typename Data_t>
    common::uint32 transmit_polling(common::uint16 a_slave_address,
                                 const Data_t& a_data,
                                 common::time_tick a_timeout_ms,
                                 Bus_status* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(a_slave_address,
                                            static_cast<void*>(&a_data),
                                            sizeof(a_data),
                                            a_timeout_ms,
                                            a_p_status);
    }

    template<typename Data_t>
    common::uint32 receive_polling(common::uint16 a_slave_address,
                                   Data_t* a_p_data,
                                   Bus_status* a_p_status = nullptr)
    {
        return this->receive_bytes_polling(a_slave_address, static_cast<void*>(a_p_data), sizeof(Data_t), a_p_status);
    }

    template<typename Data_t>
    common::uint32 receive_polling(common::uint16 a_slave_address,
                                   Data_t* a_p_data,
                                   common::time_tick a_timeout_ms,
                                   Bus_status* a_p_status = nullptr)
    {
        return this->receive_bytes_polling(a_slave_address,
                                           static_cast<void*>(&a_p_data),
                                           sizeof(Data_t),
                                           a_timeout_ms,
                                           a_p_status);
    }

    common::uint32 transmit_bytes_polling(common::uint16 a_slave_address,
                                          const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          Bus_status* a_p_status = nullptr);

    common::uint32 transmit_bytes_polling(common::uint16 a_slave_address,
                                          const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          common::time_tick a_timeout_ms,
                                          Bus_status* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(common::uint16 a_slave_address,
                                         void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         Bus_status* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(common::uint16 a_slave_address,
                                         void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         common::time_tick a_timeout_ms,
                                         Bus_status* a_p_status = nullptr);

    void start_transmit_bytes_it(common::uint16 a_slave_address,
                                 const TX_callback& a_callback,
                                 common::uint32 a_data_size_in_bytes);

    void start_receive_bytes_it(common::uint16 a_slave_address,
                                const RX_callback& a_callback,
                                common::uint32 a_data_size_in_bytes);

    void stop_transmit_bytes_it();
    void stop_receive_bytes_it();

    bool is_slave_present(common::uint16 a_slave_address, common::time_tick a_timeout_ms) const;

    bool is_analog_filter() const
    {
        assert(nullptr != this->p_i2c);

        return false == common::is_flag(p_i2c->CR1, I2C_CR1_ANFOFF);
    }

    common::uint32 get_timing() const
    {
        assert(nullptr != this->p_i2c);

        return this->p_i2c->TIMINGR;
    }

    Clock_source get_clock_source() const
    {
        return Clock_source::hsi;
    }

    bool is_fast_plus() const
    {
        return common::get_bit(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<common::uint32>(this->id));
    }

    Id get_id() const
    {
        return this->id;
    }

private:

    struct IT_context
    {
        common::uint32 index = 0;
        common::uint32 size  = 0;
    };

private:

    void clear_error_flags();
    bool is_error() const;
    Bus_status isr_to_bus_status(common::uint32 a_isr) const;

private:

    Id id;
    I2C_TypeDef* p_i2c;

    RX_callback rx_callback;
    TX_callback tx_callback;

    IT_context rx_context;
    IT_context tx_context;

private:

    friend void i2c_handle_interrupt(I2C_master* a_p_this);
};

class I2C_slave : private common::Non_copyable
{
public:

    enum class Id : common::uint32
    {
        _1,
        _2,
        _3,
        _4
    };

public:

    I2C_slave(Id a_id)
        : id(a_id)
    {}

    ~I2C_slave()
    {
        this->diasble();
    }

    void enable(bool a_analog_filter, common::uint32 a_timings, bool a_is_fast_plus);
    void diasble();

    template<typename Data_t>
    void write_polling(const Data_t& a_data)
    {
        this->write_bytes_polling(static_cast<void*>(a_data), sizeof(a_data));
    }

    template<typename Data_t>
    bool write_polling(const Data_t& a_data, common::time_tick a_timeout_ms)
    {
        return this->write_bytes_polling(static_cast<void*>(a_data), sizeof(a_data), a_timeout_ms);
    }

    template<typename Data_t>
    void read_polling(Data_t* a_p_data)
    {
        this->read_bytes_polling(static_cast<void*>(a_p_data), sizeof(Data_t));
    }

    template<typename Data_t>
    bool read_polling(Data_t* a_p_data, common::time_tick a_timeout_ms)
    {
        return this->write_bytes_polling(static_cast<void*>(&a_p_data), sizeof(Data_t), a_timeout_ms);
    }

    void write_bytes_polling(const void* a_p_data, common::uint32 a_data_size_in_bytes);
    bool write_bytes_polling(const void* a_p_data, common::uint32 a_data_size_in_bytes, common::time_tick a_timeout_ms);

    void read_bytes_polling(void* a_p_data, common::uint32 a_data_size_in_bytes);
    bool read_bytes_polling(void* a_p_data, common::uint32 a_data_size_in_bytes, common::time_tick a_timeout_ms);

    bool is_analog_filter() const
    {
        assert(nullptr != this->p_i2c);

        return false == common::is_flag(p_i2c->CR1, I2C_CR1_ANFOFF);
    }

    common::uint32 get_timing() const
    {
        assert(nullptr != this->p_i2c);

        return this->p_i2c->TIMINGR;
    }

    bool is_fast_plus() const
    {
        return common::get_bit(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<common::uint32>(this->id));
    }

    Id get_id() const
    {
        return this->id;
    }

private:

    Id id;
    I2C_TypeDef* p_i2c;

private:

    friend void i2c_handle_interrupt(I2C_slave* a_p_this);
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml