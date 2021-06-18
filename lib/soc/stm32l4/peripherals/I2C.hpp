#pragma once

/*
 *   Name: I2C.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <type_traits>

// soc
#include <soc/stm32l4/rcc.hpp>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/various.hpp>

namespace soc {
namespace stm32l4 {
namespace peripherals {

#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
    defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
    defined(STM32L452xx) || defined(STM32L462xx)

class I2C_base : private cml::Non_copyable
{
public:
    enum class Id : uint32_t
    {
        _1,
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L433xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        _2,
#endif
        _3,
#if defined(STM32L451xx) || defined(STM32L452xx) || defined(STM32L462xx)
        _4
#endif
    };

    enum class Bus_flag : uint32_t
    {
        ok               = 0x0,
        crc_error        = 0x1,
        buffer_error     = 0x2,
        arbitration_lost = 0x4,
        misplaced        = 0x8,
        nack             = 0x10,
    };

    struct Result
    {
        Bus_flag bus_flag             = cml::various::enum_incorrect_value<Bus_flag>();
        uint32_t data_length_in_bytes = 0;
    };

public:
    bool is_enabled() const;

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

constexpr I2C_base::Bus_flag operator|(I2C_base::Bus_flag a_f1, I2C_base::Bus_flag a_f2)
{
    return static_cast<I2C_base::Bus_flag>(static_cast<uint32_t>(a_f1) | static_cast<uint32_t>(a_f2));
}
constexpr I2C_base::Bus_flag operator&(I2C_base::Bus_flag a_f1, I2C_base::Bus_flag a_f2)
{
    return static_cast<I2C_base::Bus_flag>(static_cast<uint32_t>(a_f1) & static_cast<uint32_t>(a_f2));
}
constexpr I2C_base::Bus_flag operator|=(I2C_base::Bus_flag& a_f1, I2C_base::Bus_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

class I2C_master : public I2C_base
{
public:
    using Id     = I2C_base::Id;
    using Result = I2C_base::Result;

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
        using Function = void (*)(Bus_flag a_bus_flag, I2C_master* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Config
    {
        enum class Analog_filter
        {
            disabled,
            enabled
        };

        enum class Fast_plus
        {
            disabled,
            enabled
        };

        enum class Crc
        {
            disabled,
            enabled
        };

        Analog_filter analog_filter = cml::various::enum_incorrect_value<Analog_filter>();
        Fast_plus fast_plus         = cml::various::enum_incorrect_value<Fast_plus>();
        Crc crc                     = cml::various::enum_incorrect_value<Crc>();
        uint32_t timings            = 0;
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

    void enable(const Config& a_config, uint32_t a_irq_priority);
    void diasble();

    template<typename Data_t> Result transmit_polling(uint8_t a_slave_address, const Data_t& a_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(a_slave_address, &a_data, sizeof(a_data));
    }

    template<typename Data_t> Result transmit_polling(uint8_t a_slave_address, const Data_t& a_data, uint32_t a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(a_slave_address, &a_data, sizeof(a_data), a_timeout);
    }

    template<typename Data_t> Result receive_polling(uint8_t a_slave_address, Data_t* a_p_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_slave_address, a_p_data, sizeof(Data_t));
    }

    template<typename Data_t> Result receive_polling(uint8_t a_slave_address, Data_t* a_p_data, uint32_t a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_slave_address, a_p_data, sizeof(Data_t), a_timeout);
    }

    Result transmit_bytes_polling(uint8_t a_slave_address, const void* a_p_data, uint32_t a_data_size_in_bytes);
    Result transmit_bytes_polling(uint8_t a_slave_address,
                                  const void* a_p_data,
                                  uint32_t a_data_size_in_bytes,
                                  uint32_t a_timeout);

    Result receive_bytes_polling(uint8_t a_slave_address, void* a_p_data, uint32_t a_data_size_in_bytes);
    Result
    receive_bytes_polling(uint8_t a_slave_address, void* a_p_data, uint32_t a_data_size_in_bytes, uint32_t a_timeout);

    void register_transmit_callback(uint8_t a_slave_address,
                                    const Transmit_callback& a_callback,
                                    uint32_t a_data_length_in_bytes);
    void register_receive_callback(uint8_t a_slave_address,
                                   const Receive_callback& a_callback,
                                   uint32_t a_data_length_in_bytes);
    void register_bus_status_callback(const Bus_status_callback& a_callback);

    void unregister_transmit_callback();
    void unregister_receive_callback();
    void unregister_bus_status_callback();

    bool is_slave_connected(uint8_t a_slave_address, uint32_t a_timeout) const;

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

    Config get_config() const;

private:
    friend void i2c_master_interrupt_handler(I2C_master* a_p_this);

private:
    Transmit_callback transmit_callback;
    Receive_callback receive_callback;
    Bus_status_callback bus_status_callback;
};

class I2C_slave : public I2C_base
{
public:
    using Id     = I2C_base::Id;
    using Result = I2C_base::Result;

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
        using Function = void (*)(Bus_flag a_bus_flag, I2C_slave* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Addres_match_callback
    {
        using Function = void (*)(I2C_slave* a_p_this, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Config
    {
        enum class Analog_filter : uint32_t
        {
            disabled,
            enabled,
        };

        enum class Fast_plus : uint32_t
        {
            disabled,
            enabled,
        };

        enum class Crc : uint32_t
        {
            disabled,
            enabled,
        };

        Analog_filter analog_filter = cml::various::enum_incorrect_value<Analog_filter>();
        Fast_plus fast_plus         = cml::various::enum_incorrect_value<Fast_plus>();
        Crc crc                     = cml::various::enum_incorrect_value<Crc>();
        uint32_t timings            = 0;
        uint16_t address            = 0;
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

    void enable(const Config& a_config, uint32_t a_irq_priority);

    void diasble();

    template<typename Data_t> Result transmit_polling(const Data_t& a_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data));
    }

    template<typename Data_t> Result transmit_polling(const Data_t& a_data, uint32_t a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->transmit_bytes_polling(&a_data, sizeof(a_data), a_timeout);
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t));
    }

    template<typename Data_t> Result receive_polling(Data_t* a_p_data, uint32_t a_timeout)
    {
        static_assert(true == std::is_standard_layout<Data_t>::value && true == std::is_trivial<Data_t>::value);
        return this->receive_bytes_polling(a_p_data, sizeof(Data_t), a_timeout);
    }

    Result transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_bytes);
    Result transmit_bytes_polling(const void* a_p_data, uint32_t a_data_size_in_bytes, uint32_t a_timeout);

    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes);
    Result receive_bytes_polling(void* a_p_data, uint32_t a_data_size_in_bytes, uint32_t a_timeout);

    void register_transmit_callback(const Transmit_callback& a_callback);
    void register_receive_callback(const Receive_callback& a_callback);
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

    Config get_config() const;

private:
    friend void i2c_slave_interrupt_handler(I2C_slave* a_p_this);

private:
    Transmit_callback transmit_callback;
    Receive_callback receive_callback;
    Bus_status_callback bus_status_callback;
    Addres_match_callback address_match_callback;
};

#endif

} // namespace peripherals
} // namespace stm32l4
} // namespace soc

namespace soc {
namespace stm32l4 {
template<> struct rcc<peripherals::I2C_base>
{
    enum class Clock_source : uint32_t
    {
        pclk1  = 0,
        sysclk = 1,
        hsi    = 2
    };

    static void enable(peripherals::I2C_base::Id a_id, Clock_source a_clock_source, bool a_enable_in_lp);
    static void disable(peripherals::I2C_base::Id a_id);
};
} // namespace stm32l4
} // namespace soc
