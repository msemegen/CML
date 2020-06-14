#pragma once

/*
    Name: I2C.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//externals
#include <stm32l4xx.h>

//cml
#include <common/bit.hpp>
#include <common/integer.hpp>
#include <common/Non_copyable.hpp>
#include <common/time.hpp>
#include <debug/assert.hpp>

namespace cml {
namespace hal {
namespace stm32l452xx {

class I2C_base : private common::Non_copyable
{
public:

    enum class Id : common::uint32
    {
        _1,
        _2,
        _3,
        _4
    };

    enum class Clock_source : common::uint32
    {
        pclk1  = 0,
        sysclk = 1,
        hsi    = 2
    };

    enum class Bus_status_flag : common::uint32
    {
        ok               = 0x0,
        crc_error        = 0x1,
        buffer_error     = 0x2,
        arbitration_lost = 0x4,
        misplaced        = 0x8,
        nack             = 0x10,
    };

    struct TX_callback
    {
        using Function = void(*)(volatile common::uint32* a_p_data, bool a_stop, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct RX_callback
    {
        using Function = void(*)(common::uint8 a_data, bool a_stop, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_status_callback
    {
        using Function = bool(*)(Bus_status_flag a_bus_status, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:

    Clock_source get_clock_source() const;

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
        assert(nullptr != this->p_i2c);

        return common::get_bit(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP_Pos + static_cast<common::uint32>(this->id));
    }

    bool is_enabled() const
    {
        assert(nullptr != this->p_i2c);

        return common::is_flag(this->p_i2c->CR1, I2C_CR1_PE);
    }

    Id get_id() const
    {
        return this->id;
    }

protected:

    I2C_base(Id a_id)
        : id(a_id)
        , p_i2c(nullptr)
    {}

    void bus_status_interrupt_handler(common::uint32 a_isr);
    void rxne_interrupt_handler(common::uint32 a_isr, common::uint32 a_cr1);
    void txe_interrupt_handler(common::uint32 a_isr, common::uint32 a_cr1);
    void stopf_interrupt_handler(common::uint32 a_isr, common::uint32 a_cr1);

protected:

    Id id;
    mutable I2C_TypeDef* p_i2c;

    RX_callback rx_callback;
    TX_callback tx_callback;
    Bus_status_callback bus_status_callback;
};

constexpr I2C_base::Bus_status_flag operator | (I2C_base::Bus_status_flag a_f1, I2C_base::Bus_status_flag a_f2)
{
    return static_cast<I2C_base::Bus_status_flag>(static_cast<common::uint32>(a_f1) | static_cast<common::uint32>(a_f2));
}

constexpr I2C_base::Bus_status_flag operator & (I2C_base::Bus_status_flag a_f1, I2C_base::Bus_status_flag a_f2)
{
    return static_cast<I2C_base::Bus_status_flag>(static_cast<common::uint32>(a_f1) & static_cast<common::uint32>(a_f2));
}

constexpr I2C_base::Bus_status_flag operator |= (I2C_base::Bus_status_flag& a_f1, I2C_base::Bus_status_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

class I2C_master : public I2C_base
{
public:

    using Id              = I2C_base::Id;
    using Clock_source    = I2C_base::Clock_source;
    using Bus_status_flag = I2C_base::Bus_status_flag;

    using TX_callback         = I2C_base::TX_callback;
    using RX_callback         = I2C_base::RX_callback;
    using Bus_status_callback = I2C_base::Bus_status_callback;

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
                                    Bus_status_flag* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(a_slave_address, static_cast<void*>(a_data), sizeof(a_data), a_p_status);
    }

    template<typename Data_t>
    common::uint32 transmit_polling(common::uint16 a_slave_address,
                                 const Data_t& a_data,
                                 common::time::tick a_timeout,
                                 Bus_status_flag* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(a_slave_address,
                                            static_cast<void*>(&a_data),
                                            sizeof(a_data),
                                            a_timeout,
                                            a_p_status);
    }

    template<typename Data_t>
    common::uint32 receive_polling(common::uint16 a_slave_address,
                                   Data_t* a_p_data,
                                   Bus_status_flag* a_p_status = nullptr)
    {
        return this->receive_bytes_polling(a_slave_address, static_cast<void*>(a_p_data), sizeof(Data_t), a_p_status);
    }

    template<typename Data_t>
    common::uint32 receive_polling(common::uint16 a_slave_address,
                                   Data_t* a_p_data,
                                   common::time::tick a_timeout,
                                   Bus_status_flag* a_p_status = nullptr)
    {
        return this->receive_bytes_polling(a_slave_address,
                                           static_cast<void*>(&a_p_data),
                                           sizeof(Data_t),
                                           a_timeout,
                                           a_p_status);
    }

    common::uint32 transmit_bytes_polling(common::uint16 a_slave_address,
                                          const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          Bus_status_flag* a_p_status = nullptr);

    common::uint32 transmit_bytes_polling(common::uint16 a_slave_address,
                                          const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          common::time::tick a_timeout,
                                          Bus_status_flag* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(common::uint16 a_slave_address,
                                         void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         Bus_status_flag* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(common::uint16 a_slave_address,
                                         void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         common::time::tick a_timeout,
                                         Bus_status_flag* a_p_status = nullptr);

    void register_transmit_callback(common::uint16 a_slave_address,
                                    const TX_callback& a_callback,
                                    common::uint32 a_data_size_in_bytes);

    void register_receive_callback(common::uint16 a_slave_address,
                                   const RX_callback& a_callback,
                                   common::uint32 a_data_size_in_bytes);

    void register_bus_status_callback(const Bus_status_callback& a_callback);
    void unregister_bus_status_callback();

    bool is_slave_connected(common::uint16 a_slave_address, common::time::tick a_timeout) const;

 private:

     friend void i2c_master_interrupt_handler(I2C_master* a_p_this);
};

class I2C_slave : public I2C_base
{
public:

    using Id              = I2C_base::Id;
    using Clock_source    = I2C_base::Clock_source;
    using Bus_status_flag = I2C_base::Bus_status_flag;

    using TX_callback         = I2C_base::TX_callback;
    using RX_callback         = I2C_base::RX_callback;
    using Bus_status_callback = I2C_base::Bus_status_callback;

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
    common::uint32 transmit_polling(const Data_t& a_data, Bus_status_flag* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(static_cast<void*>(a_data), sizeof(a_data), a_p_status);
    }

    template<typename Data_t>
    common::uint32 write_polling(const Data_t& a_data,
                                 common::time::tick a_timeout,
                                 Bus_status_flag* a_p_status = nullptr)
    {
        return this->transmit_bytes_polling(static_cast<void*>(a_data), sizeof(a_data), a_timeout, a_p_status);
    }

    template<typename Data_t>
    common::uint32 receive_polling(Data_t* a_p_data, Bus_status_flag* a_p_status = nullptr)
    {
        return this->receive_bytes_polling(static_cast<void*>(a_p_data), sizeof(Data_t), a_p_status);
    }

    template<typename Data_t>
    bool receive_polling(Data_t* a_p_data, common::time::tick a_timeout, Bus_status_flag* a_p_status)
    {
        return this->receive_bytes_polling(static_cast<void*>(&a_p_data), sizeof(Data_t), a_timeout, a_p_status);
    }

    common::uint32 transmit_bytes_polling(const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          Bus_status_flag* a_p_status = nullptr);

    common::uint32 transmit_bytes_polling(const void* a_p_data,
                                          common::uint32 a_data_size_in_bytes,
                                          common::time::tick a_timeout,
                                          Bus_status_flag* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         Bus_status_flag* a_p_status = nullptr);

    common::uint32 receive_bytes_polling(void* a_p_data,
                                         common::uint32 a_data_size_in_bytes,
                                         common::time::tick a_timeout,
                                         Bus_status_flag* a_p_status = nullptr);

    void register_transmit_callback(const TX_callback& a_callback,
                                    common::uint32 a_data_size_in_bytes);

    void register_receive_callback(const RX_callback& a_callback,
                                   common::uint32 a_data_size_in_bytes);

    void register_bus_status_callback(const Bus_status_callback& a_callback);
    void unregister_bus_status_callback();

private:

    friend void i2c_slave_interrupt_handler(I2C_slave* a_p_this);
};

} // namespace stm32l452xx
} // namespace hal
} // namespace cml