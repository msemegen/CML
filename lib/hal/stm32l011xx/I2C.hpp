#pragma once

/*
    Name: I2C.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l0xx.h>

//cml
#include <common/bit.hpp>
#include <common/integer.hpp>
#include <common/macros.hpp>
#include <common/Non_copyable.hpp>
#include <common/time_tick.hpp>
#include <debug/assert.hpp>

namespace cml {
namespace hal {
namespace stm32l011xx {

class I2C_base : private common::Non_copyable
{
public:

    enum class Id : common::uint32
    {
        _1,
    };

    enum class Clock_source
    {
        pclk1 = 0,
        sysclk = 1,
        hsi = 2
    };

    enum class Bus_status
    {
        ok,
        timeout,
        overrun,
        underrun,
        arbitration_lost,
        misplaced,
        crc_error,
        nack
    };

    struct TX_callback
    {
        using Function = bool(*)(volatile common::uint32* a_p_data, Bus_status a_status, void* a_p_user_data);

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

    Clock_source get_clock_source() const;

    bool is_analog_filter() const
    {
        return false == common::is_flag(I2C1->CR1, I2C_CR1_ANFOFF);
    }

    common::uint32 get_timing() const
    {
        return I2C1->TIMINGR;
    }

    bool is_fast_plus() const
    {
        return common::get_bit(SYSCFG->CFGR2, SYSCFG_CFGR2_I2C1_FMP_Pos);
    }

    bool is_enabled() const
    {
        return common::is_flag(I2C1->CR1, I2C_CR1_PE);
    }

    Id get_id() const
    {
        return Id::_1;
    }

protected:

    I2C_base(Id a_id)
    {
        unused(a_id);
    }

protected:

    struct IT_context
    {
        common::uint32 index = 0;
        common::uint32 size = 0;
    };

protected:

    RX_callback rx_callback;
    TX_callback tx_callback;

    IT_context rx_context;
    IT_context tx_context;

protected:

    friend void i2c_handle_interrupt(I2C_base* a_p_this);
};

class I2C_master : public I2C_base
{
public:

    using Id           = I2C_base::Id;
    using Clock_source = I2C_base::Clock_source;
    using Bus_status   = I2C_base::Bus_status;

    using TX_callback = I2C_base::TX_callback;
    using RX_callback = I2C_base::RX_callback;

    struct Config
    {
        bool analog_filter     = false;
        bool fast_plus         = false;
        bool crc_enable        = false;
        common::uint32 timings = 0;
    };

public:

    I2C_master(Id a_id)
        : I2C_base(a_id)
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

    bool is_slave_connected(common::uint16 a_slave_address, common::time_tick a_timeout_ms) const;

};

class I2C_slave : public I2C_base
{
public:

    using Id           = I2C_base::Id;
    using Clock_source = I2C_base::Clock_source;
    using Bus_status   = I2C_base::Bus_status;

    using TX_callback = I2C_base::TX_callback;
    using RX_callback = I2C_base::RX_callback;

    struct Config
    {
        bool analog_filter     = false;
        bool fast_plus         = false;
        bool crc_enable        = false;
        common::uint32 timings = 0;
        common::uint16 address = 0;
    };

public:

    I2C_slave(Id a_id)
        : I2C_base(a_id)
    {}

    ~I2C_slave()
    {
        this->diasble();
    }

    void enable(const Config& a_config,
                Clock_source a_clock_source,
                common::uint32 a_irq_priority);

    void diasble();

    template<typename Data_t>
    common::uint32 transmit_polling(const Data_t& a_data, Bus_status* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(static_cast<void*>(a_data), sizeof(a_data), a_p_status);
    }

    template<typename Data_t>
    common::uint32 write_polling(const Data_t& a_data, common::time_tick a_timeout_ms, Bus_status* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(static_cast<void*>(a_data), sizeof(a_data), a_timeout_ms, a_p_status);
    }

    template<typename Data_t>
    common::uint32 receive_polling(Data_t* a_p_data, Bus_status* a_p_status = nullptr)
    {
        return this->receive_bytes_polling(static_cast<void*>(a_p_data), sizeof(Data_t), a_p_status);
    }

    template<typename Data_t>
    bool receive_polling(Data_t* a_p_data, common::time_tick a_timeout_ms, Bus_status* a_p_status)
    {
        return this->receive_bytes_polling(static_cast<void*>(&a_p_data), sizeof(Data_t), a_timeout_ms, a_p_status);
    }

    common::uint32 transmit_bytes_polling(const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          Bus_status* a_p_status = nullptr);

    common::uint32 transmit_bytes_polling(const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          common::time_tick a_timeout_ms,
                                          Bus_status* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         Bus_status* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         common::time_tick a_timeout_ms,
                                         Bus_status* a_p_status = nullptr);

    void stop_transmit_bytes_it();
    void stop_receive_bytes_it();
};

} // namespace stm32l011xx
} // namespace hal
} // namespace cml