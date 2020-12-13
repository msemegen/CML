#pragma once

/*
    Name: LSM6DSL.hpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// std
#include <cstdint>

// cml
#include <cml/Non_copyable.hpp>
#include <cml/debug/assert.hpp>
#include <cml/time.hpp>

namespace cml {
namespace devices {

class LSM6DSL : private Non_copyable
{
public:
    template<typename Type_t> struct Axis
    {
        Type_t x = static_cast<Type_t>(0);
        Type_t y = static_cast<Type_t>(0);
        Type_t z = static_cast<Type_t>(0);

        Type_t operator[](uint32_t a_index) const
        {
            assert(a_index < 3);
            static_assert(sizeof(*this) == sizeof(Type_t) * 3);
            return (reinterpret_cast<const Type_t*>(this))[a_index];
        }

        Type_t& operator[](uint32_t a_index)
        {
            assert(a_index < 3);
            static_assert(sizeof(*this) == sizeof(Type_t) * 3);
            return (reinterpret_cast<Type_t*>(this))[a_index];
        }
    };

    struct Transmit
    {
        using Function = bool (*)(uint8_t a_register, uint8_t a_value, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;

        bool operator()(uint8_t a_register, uint8_t a_value) const
        {
            assert(nullptr != this->function);
            return this->function(a_register, a_value, this->p_user_data);
        }
    };

    struct Receive
    {
        using Function = bool (*)(uint8_t a_register, uint8_t* a_p_data, uint32_t a_size_in_bytes, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;

        bool operator()(uint8_t a_register, uint8_t* a_p_data, uint32_t a_size_in_bytes) const
        {
            assert(nullptr != this->function);
            return this->function(a_register, a_p_data, a_size_in_bytes, this->p_user_data);
        }
    };

    class Accelerometer : private Non_copyable
    {
    public:
        enum class Fullscale_range : uint8_t
        {
            _2g     = 0x0u,
            _16g    = 0x4u,
            _4g     = 0x8u,
            _8g     = 0xCu,
            unknown = 0xFFu
        };

        enum class Output_data_rate : uint8_t
        {
            _12_5_Hz = 0x10u,
            _26_Hz   = 0x20u,
            _52_Hz   = 0x30u,
            _104_Hz  = 0x40u,
            _208_Hz  = 0x50u,
            _416_Hz  = 0x60u,
            _833_Hz  = 0x70u,
            _1660_Hz = 0x80u,
            _3330_Hz = 0x90u,
            _6660_Hz = 0xA0u,
            _1_6_Hz  = 0xB0u,
            unknown  = 0xFFu
        };

        enum class Power_mode : uint8_t
        {
            normal,
            high_performance,
            unknown
        };

    public:
        bool enable(Fullscale_range a_fullscale_range, Output_data_rate a_output_data_rate, Power_mode a_power_mode);
        bool disable();

        bool set_fullscale_range(Fullscale_range a_fullscale_range);
        bool set_output_data_rate(Output_data_rate a_output_data_rate);
        bool set_power_mode(Power_mode a_power_mode);

        bool get_axis_raw_polling(Axis<int16_t>* a_p_axis) const;
        bool get_axis_scaled_polling(Axis<float>* a_p_axis) const;

        bool get_axis_raw_polling(Axis<int16_t>* a_p_axis, time::tick a_timeout) const;
        bool get_axis_scaled_polling(Axis<float>* a_p_axis, time::tick a_timeout) const;

        bool is_enabled(bool* a_p_flag) const;

        Fullscale_range get_fullscale_range() const
        {
            return this->fullscale_range;
        }

        Output_data_rate get_output_data_rate() const
        {
            return this->output_data_rate;
        }

        Power_mode get_power_mode() const
        {
            return this->power_mode;
        }

    private:
        Accelerometer(LSM6DSL* a_p_owner)
            : p_owner(a_p_owner)
            , fullscale_range(Fullscale_range::unknown)
            , output_data_rate(Output_data_rate::unknown)
            , power_mode(Power_mode::unknown)
        {
        }

        ~Accelerometer()
        {
            this->disable();
        }

    private:
        LSM6DSL* p_owner;

        Fullscale_range fullscale_range;
        Output_data_rate output_data_rate;
        Power_mode power_mode;

        friend LSM6DSL;
    };

    class Gyroscope : private Non_copyable
    {
    public:
        enum class Fullscale_range : uint8_t
        {
            _250_dps  = 0x0u,
            _500_dps  = 0x4u,
            _1000_dps = 0x8u,
            _2000_dps = 0xCu,
            _125_dps  = 0xDu,
            unknown   = 0xFFu
        };

        enum class Output_data_rate : uint8_t
        {
            _12_5_Hz = 0x10u,
            _26_Hz   = 0x20u,
            _52_Hz   = 0x30u,
            _104_Hz  = 0x40u,
            _208_Hz  = 0x50u,
            _416_Hz  = 0x60u,
            _833_Hz  = 0x70u,
            _1660_Hz = 0x80u,
            _3330_Hz = 0x90u,
            _6660_Hz = 0xA0u,
            unknown  = 0xFFu
        };

        enum class Power_mode : uint8_t
        {
            normal,
            high_performance,
            unknown
        };

    public:
        bool enable(Fullscale_range a_fullscale_range, Output_data_rate a_output_data_rate, Power_mode a_power_mode);
        bool disable();

        bool set_fullscale_range(Fullscale_range a_fullscale_range);
        bool set_output_data_rate(Output_data_rate a_output_data_rate);
        bool set_power_mode(Power_mode a_power_mode);

        bool get_axis_raw_polling(Axis<int16_t>* a_p_axis);
        bool get_axis_scaled_polling(Axis<float>* a_p_axis);

        bool get_axis_raw_polling(Axis<int16_t>* a_p_axis, time::tick a_timeout);
        bool get_axis_scaled_polling(Axis<float>* a_p_axis, time::tick a_timeout);

        bool is_enabled(bool* a_p_flag) const;

        Fullscale_range get_fullscale_range() const
        {
            return this->fullscale_range;
        }

        Output_data_rate get_output_data_rate() const
        {
            return this->output_data_rate;
        }

        Power_mode get_power_mode() const
        {
            return this->power_mode;
        }

    private:
        Gyroscope(LSM6DSL* a_p_owner)
            : p_owner(a_p_owner)
            , fullscale_range(Fullscale_range::unknown)
            , output_data_rate(Output_data_rate::unknown)
            , power_mode(Power_mode::unknown)
        {
        }

        ~Gyroscope()
        {
            this->disable();
        }

    private:
        LSM6DSL* p_owner;

        Fullscale_range fullscale_range;
        Output_data_rate output_data_rate;
        Power_mode power_mode;

        friend LSM6DSL;
    };

    class Temperature : private Non_copyable
    {
    public:
        bool get_data_raw_polling(int16_t* a_p_temperature) const;
        bool get_data_scaled_polling(float* a_p_temperature) const;

        bool get_data_raw_polling(int16_t* a_p_temperature, time::tick a_timeout) const;
        bool get_data_scaled_polling(float* a_p_temperature, time::tick a_timeout) const;

    private:
        Temperature(LSM6DSL* a_p_owner)
            : p_owner(a_p_owner)
        {
        }

        ~Temperature() = default;

    private:
        LSM6DSL* p_owner;

        friend LSM6DSL;
    };

    class Fifo : private Non_copyable
    {
    public:
        enum class Output_data_rate : uint8_t
        {
            _12_5_Hz = 0x8u,
            _26_Hz   = 0x10u,
            _52_Hz   = 0x18u,
            _104_Hz  = 0x20u,
            _208_Hz  = 0x28u,
            _416_Hz  = 0x30u,
            _833_Hz  = 0x38u,
            _1666_Hz = 0x40u,
            _3333_Hz = 0x48u,
            _6666_Hz = 0x50u,
            unknown  = 0xFFu
        };

    public:
        bool enable(Output_data_rate a_output_data_rate, uint32_t a_pattern_count);
        bool disable();

        bool set_output_data_rate(Output_data_rate a_output_data_rate);

        bool get_data_raw_polling(Axis<int16_t>* a_p_gyroscope_data,
                                  uint32_t a_gyroscope_data_buffer_capacity,
                                  Axis<int16_t>* a_p_accelerometer_data,
                                  uint32_t a_accelerometer_data_buffer_capacity);

        bool get_data_scaled_polling(Axis<float>* a_p_gyroscope_data,
                                     uint32_t a_gyroscope_data_buffer_capacity,
                                     Axis<float>* a_p_accelerometer_data,
                                     uint32_t a_accelerometer_data_buffer_capacity);

        bool get_data_raw_polling(Axis<int16_t>* a_p_gyroscope_data,
                                  uint32_t a_gyroscope_data_buffer_capacity,
                                  Axis<int16_t>* a_p_accelerometer_data,
                                  uint32_t a_accelerometer_data_buffer_capacity,
                                  time::tick a_timeout);

        bool get_data_scaled_polling(Axis<float>* a_p_gyroscope_data,
                                     uint32_t a_gyroscope_data_buffer_capacity,
                                     Axis<float>* a_p_accelerometer_data,
                                     uint32_t a_accelerometer_data_buffer_capacity,
                                     time::tick a_timeout);

        bool is_enabled(bool* a_p_flag) const;

        Output_data_rate get_output_data_rate() const
        {
            return this->output_data_rate;
        }

    private:
        Fifo(LSM6DSL* a_p_owner)
            : p_owner(a_p_owner)
            , pattern_count(0)
            , output_data_rate(Output_data_rate::unknown)
        {
        }

        ~Fifo()
        {
            this->disable();
        }

    private:
        struct Pattern
        {
            uint32_t gyroscope     = 0;
            uint32_t accelerometer = 0;
        };

    private:
        bool is_full(bool* a_p_flag) const;

        uint32_t output_data_rate_to_Hz(Accelerometer::Output_data_rate a_output_data_rate) const;
        uint32_t output_data_rate_to_Hz(Gyroscope::Output_data_rate a_output_data_rate) const;

        uint8_t decimation_to_ctrl3_register(uint32_t a_accelerometer_decimation,
                                             uint32_t a_gyroscope_decimation) const;

    private:
        LSM6DSL* p_owner;

        Pattern pattern;
        uint32_t pattern_count;
        Output_data_rate output_data_rate;

        friend LSM6DSL;
    };

public:
    LSM6DSL(const Transmit& a_transmit, const Receive& a_receive)
        : transmit(a_transmit)
        , receive(a_receive)
        , accelerometer(this)
        , gyroscope(this)
        , temperature(this)
        , fifo(this)
    {
    }

    ~LSM6DSL()
    {
        this->disable();
    }

    bool enable();
    bool disable();

    bool reboot(bool* a_p_flag);
    bool get_id(uint8_t* a_p_id) const;

public:
    Accelerometer* const p_accelerometer = &(this->accelerometer);
    Gyroscope* const p_gyroscope         = &(this->gyroscope);
    Temperature* const p_temperature     = &(this->temperature);
    Fifo* const p_fifo                   = &(this->fifo);

private:
    enum class Sensor
    {
        accelerometer = 0,
        gyroscope     = 1,
        temperature   = 2
    };

private:
    bool get_data(uint8_t a_register, void* a_p_buffer, uint32_t a_buffer_capacity) const;
    bool is_data_avaliable(Sensor a_data, bool* a_p_flag) const;

private:
    Transmit transmit;
    Receive receive;

    Accelerometer accelerometer;
    Gyroscope gyroscope;
    Temperature temperature;
    Fifo fifo;
};

} // namespace devices
} // namespace cml