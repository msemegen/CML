#pragma once

/*
    Name: I2C.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>
#include <type_traits>

// externals
#include <stm32l0xx.h>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/bit.hpp>
#include <cml/bit_flag.hpp>
#include <cml/debug/assert.hpp>
#include <cml/time.hpp>

namespace soc {
namespace stm32l011xx {
namespace peripherals {

class I2C_base : private cml::Non_copyable
{
public:
    enum class Id : uint32_t
    {
        _1,
    };

    enum class Clock_source : uint32_t
    {
        pclk1  = 0,
        sysclk = 1,
        hsi    = 2
    };

    struct Result
    {
        enum class Bus_flag : uint32_t
        {
            ok               = 0x0,
            crc_error        = 0x1,
            buffer_error     = 0x2,
            arbitration_lost = 0x4,
            misplaced        = 0x8,
            nack             = 0x10,
            unknown          = 0x20
        };

        Bus_flag bus_flag             = Bus_flag::unknown;
        uint32_t data_length_in_bytes = 0;
    };

public:
    Clock_source get_clock_source() const;

    bool is_analog_filter() const
    {
        return false == cml::bit_flag::is(I2C1->CR1, I2C_CR1_ANFOFF);
    }

    uint32_t get_timing() const
    {
        return I2C1->TIMINGR;
    }

    bool is_fast_plus() const
    {
        return cml::bit::is(SYSCFG->CFGR2, SYSCFG_CFGR2_I2C1_FMP_Pos);
    }

    bool is_enabled() const
    {
        return cml::bit_flag::is(I2C1->CR1, I2C_CR1_PE);
    }

    Id get_id() const
    {
        return this->id;
    }

protected:
    I2C_base(Id a_id)
        : id(a_id)
    {
    }

protected:
    Id id;
};

constexpr I2C_base::Result::Bus_flag operator|(I2C_base::Result::Bus_flag a_f1, I2C_base::Result::Bus_flag a_f2)
{
    return static_cast<I2C_base::Result::Bus_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}

constexpr I2C_base::Result::Bus_flag operator&(I2C_base::Result::Bus_flag a_f1, I2C_base::Result::Bus_flag a_f2)
{
    return static_cast<I2C_base::Result::Bus_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}

constexpr I2C_base::Result::Bus_flag operator|=(I2C_base::Result::Bus_flag& a_f1, I2C_base::Result::Bus_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

class I2C_master : public I2C_base
{
public:
    using Id           = I2C_base::Id;
    using Clock_source = I2C_base::Clock_source;
    using Result       = I2C_base::Result;

    struct Transmit_callback
    {
        using Function = void (*)(volatile uint32_t* a_p_data, bool a_stop, I2C_master* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Receive_callback
    {
        using Function = void (*)(uint8_t a_data, bool a_stop, I2C_master* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_status_callback
    {
        using Function = void (*)(Result::Bus_flag a_bus_status, I2C_master* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Config
    {
        bool analog_filter = false;
        bool fast_plus     = false;
        bool crc_enable    = false;
        uint32_t timings   = 0;
    };

public:
    I2C_master(Id a_id)
        : I2C_base(a_id)
    {
    }

    ~I2C_master()
    {
        this->diasble();
    }

    void enable(const Config& a_config, Clock_source a_clock_source, uint32_t a_irq_priority);
    void diasble();

    template<typename Data_t> Result transmit_polling(uint8_t a_slave_address, const Data_t& a_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(a_slave_address, &a_data, sizeof(a_data));
    }

    template<typename Data_t>
    Result transmit_polling(uint8_t a_slave_address, const Data_t& a_data, cml::time::tick a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(a_slave_address, &a_data, sizeof(a_data), a_timeout);
    }

    template<typename Data_t> Result receive_polling(uint8_t a_slave_address, Data_t* a_p_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_slave_address, a_p_data, sizeof(Data_t));
    }

    template<typename Data_t>
    Result receive_polling(uint8_t a_slave_address, Data_t* a_p_data, cml::time::tick a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_slave_address, a_p_data, sizeof(Data_t), a_timeout);
    }

    Result transmit_bytes_polling(uint8_t a_slave_address, const void* a_p_data, uint32_t a_data_size_in_bytes);

    Result transmit_bytes_polling(uint8_t a_slave_address,
                                  const void* a_p_data,
                                  uint32_t a_data_size_in_bytes,
                                  cml::time::tick a_timeout);

    Result receive_bytes_polling(uint8_t a_slave_address, void* a_p_data, uint32_t a_data_size_in_bytes);

    Result receive_bytes_polling(uint8_t a_slave_address,
                                 void* a_p_data,
                                 uint32_t a_data_size_in_bytes,
                                 cml::time::tick a_timeout);

    void register_transmit_callback(uint8_t a_slave_address,
                                    const Transmit_callback& a_callback,
                                    uint32_t a_data_size_in_bytes);

    void register_receive_callback(uint8_t a_slave_address,
                                   const Receive_callback& a_callback,
                                   uint32_t a_data_size_in_bytes);

    void register_bus_status_callback(const Bus_status_callback& a_callback);

    void unregister_transmit_callback();
    void unregister_receive_callback();
    void unregister_bus_status_callback();

    bool is_slave_connected(uint8_t a_slave_address, cml::time::tick a_timeout) const;

    bool is_transmit_callback() const
    {
        return nullptr != this->transmit_callback.function;
    }

    bool is_receive_callback() const
    {
        return nullptr != this->receive_callback.function;
    }

    bool is_bus_status_callback() const
    {
        return nullptr != this->bus_status_callback.function;
    }

private:
    Transmit_callback transmit_callback;
    Receive_callback receive_callback;
    Bus_status_callback bus_status_callback;

private:
    friend void i2c_master_interrupt_handler(I2C_master* a_p_this);
};

class I2C_slave : public I2C_base
{
public:
    using Id           = I2C_base::Id;
    using Clock_source = I2C_base::Clock_source;
    using Result       = I2C_base::Result;

    struct Config
    {
        bool analog_filter = false;
        bool fast_plus     = false;
        bool crc_enable    = false;
        uint32_t timings   = 0;
        uint16_t address   = 0;
    };

    struct Transmit_callback
    {
        using Function = void (*)(volatile uint32_t* a_p_data, bool a_stop, I2C_slave* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Receive_callback
    {
        using Function = void (*)(uint8_t a_data, bool a_stop, I2C_slave* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Bus_status_callback
    {
        using Function = void (*)(Result::Bus_flag a_bus_status, I2C_slave* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Addres_match_callback
    {
        using Function = void (*)(I2C_slave* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

public:
    I2C_slave(Id a_id)
        : I2C_base(a_id)
    {
    }

    ~I2C_slave()
    {
        this->diasble();
    }

    void enable(const Config& a_config, Clock_source a_clock_source, uint32_t a_irq_priority);
    void diasble();

    template<typename Data_t> Result transmit_polling(const Data_t& a_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data));
    }

    template<typename Data_t> Result transmit_polling(const Data_t& a_data, cml::time::tick a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data), a_timeout);
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t));
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data, cml::time::tick a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t), a_timeout);
    }

    Result transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_bytes);
    Result transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_bytes, cml::time::tick a_timeout);

    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes);
    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes, cml::time::tick a_timeout);

    void register_transmit_callback(const Transmit_callback& a_callback, uint32_t a_data_size_in_bytes);
    void register_receive_callback(const Receive_callback& a_callback, uint32_t a_data_size_in_bytes);
    void register_bus_status_callback(const Bus_status_callback& a_callback);
    void register_address_match_callback(const Addres_match_callback& a_callback);

    void unregister_transmit_callback();
    void unregister_receive_callback();
    void unregister_bus_status_callback();
    void unregister_address_match_callback();

    bool is_transmit_callback() const
    {
        return nullptr != this->transmit_callback.function;
    }

    bool is_receive_callback() const
    {
        return nullptr != this->receive_callback.function;
    }

    bool is_bus_status_callback() const
    {
        return nullptr != this->bus_status_callback.function;
    }

    bool is_address_match_callback() const
    {
        return nullptr != this->address_match_callback.function;
    }

private:
    Transmit_callback transmit_callback;
    Receive_callback receive_callback;
    Bus_status_callback bus_status_callback;
    Addres_match_callback address_match_callback;

private:
    friend void i2c_slave_interrupt_handler(I2C_slave* a_p_this);
};

} // namespace peripherals
} // namespace stm32l011xx
} // namespace soc