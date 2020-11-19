/*
    Name: LSM6DSL.cpp

    Copyright(c) 2020 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

// this
#include <devices/LSM6DSL/LSM6DSL.hpp>

// std
#include <algorithm>

// cml
#include <cml/bit.hpp>
#include <cml/debug/assert.hpp>

namespace {

using namespace cml::devices;

constexpr float accelerometer_sensitivity_lut[] = { 0.000061f, 0.000488f, 0.000122f, 0.000244f };
constexpr float gyroscope_sensitivity_lut[]     = { 0.00875, 0.004375f, 0.0175f, 0.0f, 0.035f, 0.0f, 0.07f };

uint32_t odr_to_freq(LSM6DSL::Accelerometer_config::Output_data_rate a_odr)
{
    if (LSM6DSL::Accelerometer_config::Output_data_rate::unknown != a_odr)
    {
        return 0xDu << (static_cast<uint8_t>(a_odr) - 1);
    }
    return 0;
}

uint32_t odr_to_freq(LSM6DSL::Gyroscope_config::Output_data_rate a_odr)
{
    if (LSM6DSL::Gyroscope_config::Output_data_rate::unknown != a_odr)
    {
        return 0xDu << (static_cast<uint8_t>(a_odr) - 1);
    }
    return 0;
}

uint32_t freq_to_odr(uint32_t a_freq_Hz)
{
    switch (a_freq_Hz)
    {
        case 13u:
            return 0x1u;

        case 26u:
            return 0x2u;

        case 52u:
            return 0x3u;

        case 104u:
            return 0x4u;

        case 208u:
            return 0x5u;

        case 416u:
            return 0x6u;

        case 832u:
            return 0x7u;

        case 1664u:
            return 0x8u;

        case 3328u:
            return 0x9u;

        case 6656u:
            return 0xAu;
    }

    return 0x0u;
}

uint32_t decimation_to_flag(uint32_t a_decimation)
{
    if (a_decimation <= 4)
    {
        return a_decimation;
    }
    else
    {
        switch (a_decimation)
        {
            case 8: {
                return 0x5u;
            }
            break;

            case 16: {
                return 0x6u;
            }
            break;

            case 32: {
                return 0x7u;
            }
            break;
        }
    }

    return 0;
}

} // namespace

namespace cml {
namespace devices {

using namespace std;

bool LSM6DSL::enable(const Accelerometer_config& a_acc_config,
                     const Gyroscope_config& a_gyr_config,
                     const Fifo_config& a_fifo_config)
{
    assert(true == a_acc_config.enabled ?
               Accelerometer_config::Fullscale_range::unknown != a_acc_config.fullscale_range &&
                   Accelerometer_config::Output_data_rate::unknown != a_acc_config.output_data_rate :
               true);

    assert(true == a_gyr_config.enabled ?
               Gyroscope_config::Fullscale_range::unknown != a_gyr_config.fullscale_range &&
                   Gyroscope_config::Output_data_rate::unknown != a_gyr_config.output_data_rate :
               true);

    assert(true == a_fifo_config.enabled ? 0 != a_fifo_config.pattern_repetition_count : true);

    bool acc_ret  = false;
    bool gyr_ret  = false;
    bool fifo_ret = false;

    uint8_t min_odr_Hz        = 0;
    uint8_t max_odr_Hz        = 0;
    uint8_t gyr_samples_count = 0;
    uint8_t acc_samples_count = 0;
    uint8_t gyr_decimation    = 0;
    uint8_t acc_decimation    = 0;

    bool ret = this->transmit.function(Registers::ctrl3_c, 0x44u, this->transmit.p_user_data);

    if (true == ret)
    {
        ret = this->transmit.function(Registers::ctrl1_xl, 0x0u, this->transmit.p_user_data) &&
              this->transmit.function(Registers::ctrl2_g, 0x0u, this->transmit.p_user_data);
    }

    if (true == ret)
    {
        if (true == a_fifo_config.enabled)
        {
            uint8_t gyr_odr_Hz =
                odr_to_freq(true == a_gyr_config.enabled ? a_gyr_config.output_data_rate :
                                                           Gyroscope_config::Output_data_rate::unknown);
            uint8_t acc_odr_Hz =
                odr_to_freq(true == a_acc_config.enabled ? a_acc_config.output_data_rate :
                                                           Accelerometer_config::Output_data_rate::unknown);

            max_odr_Hz = max(acc_odr_Hz > 0 ? acc_odr_Hz : 0x00u, gyr_odr_Hz > 0 ? gyr_odr_Hz : 0x00u);
            min_odr_Hz = min(acc_odr_Hz > 0 ? acc_odr_Hz : 0xFFu, gyr_odr_Hz > 0 ? gyr_odr_Hz : 0xFFu);

            gyr_decimation = gyr_odr_Hz > 0 ? max_odr_Hz / gyr_odr_Hz : 0;
            acc_decimation = acc_odr_Hz > 0 ? max_odr_Hz / acc_odr_Hz : 0;

            gyr_samples_count = gyr_odr_Hz / min_odr_Hz;
            acc_samples_count = acc_odr_Hz / min_odr_Hz;

            uint8_t wtm = a_fifo_config.pattern_repetition_count * (gyr_samples_count + acc_samples_count) * 3;

            fifo_ret = this->transmit.function(Registers::fifo_ctrl1, wtm & 0xFFu, this->transmit.p_user_data) &&
                       this->transmit.function(Registers::fifo_ctrl2, (wtm & 0x700u) >> 8u, this->transmit.p_user_data);

            if (true == ret)
            {
                fifo_ret = this->transmit.function(Registers::fifo_ctrl3,
                                                   decimation_to_flag(acc_decimation) |
                                                       (decimation_to_flag(gyr_decimation) << 3u),
                                                   this->transmit.p_user_data);
            }
        }

        if (true == a_acc_config.enabled)
        {
            acc_ret = this->transmit.function(Registers::ctrl1_xl,
                                              static_cast<uint8_t>(a_acc_config.fullscale_range) |
                                                  static_cast<uint8_t>(a_acc_config.output_data_rate) << 4u,
                                              this->transmit.p_user_data);

            if (true == acc_ret)
            {
                uint8_t v = 0;
                acc_ret   = this->receive.function(Registers::ctrl6_c, &v, sizeof(v), this->receive.p_user_data);

                if (true == acc_ret)
                {
                    (a_acc_config.output_data_rate == Accelerometer_config::Output_data_rate::_1_6_Hz ||
                     a_acc_config.output_data_rate <= Accelerometer_config::Output_data_rate::_208_Hz) ?
                        set_bit(&v, 4) :
                        clear_flag(&v, 4);

                    acc_ret = this->transmit.function(Registers::ctrl6_c, v, this->transmit.p_user_data);
                }
            }
        }

        if (true == a_gyr_config.enabled)
        {
            gyr_ret = this->transmit.function(Registers::ctrl2_g,
                                              static_cast<uint8_t>(a_gyr_config.fullscale_range) |
                                                  static_cast<uint8_t>(a_gyr_config.output_data_rate) << 4u,
                                              this->transmit.p_user_data);

            if (true == gyr_ret)
            {
                uint8_t v = 0;
                gyr_ret   = this->receive.function(Registers::ctrl7_g, &v, sizeof(v), this->receive.p_user_data);

                if (true == gyr_ret)
                {
                    a_gyr_config.output_data_rate <= Gyroscope_config::Output_data_rate::_208_Hz ? set_bit(&v, 7) :
                                                                                                   clear_flag(&v, 7);
                    gyr_ret = this->transmit.function(Registers::ctrl7_g, v, this->transmit.p_user_data);
                }
            }
        }

        if (true == fifo_ret)
        {
            fifo_ret = this->transmit.function(
                Registers::fifo_ctrl5, 0x1u | (freq_to_odr(min_odr_Hz) << 3u), this->transmit.p_user_data);

            if (true == fifo_ret)
            {
                this->fifo_config = a_fifo_config;

                this->fifo_context.acc_samples_count = acc_samples_count;
                this->fifo_context.gyr_samples_count = gyr_samples_count;
                this->fifo_context.gyr_decimation    = gyr_decimation;
                this->fifo_context.acc_decimation    = acc_decimation;
                this->fifo_context.odr               = freq_to_odr(min_odr_Hz);
            }
        }

        if (true == acc_ret)
        {
            this->accelerometer_config = a_acc_config;
        }

        if (true == gyr_ret)
        {
            this->gyroscope_config = a_gyr_config;
        }
    }

    return true == ret ?
               (true == a_acc_config.enabled ? acc_ret : true) && (true == a_gyr_config.enabled ? gyr_ret : true) &&
                   (true == a_fifo_config.enabled ? fifo_ret : true) :
               false;
}

bool LSM6DSL::disable(bool a_accelerometer, bool a_groscope)
{
    bool acc_ret  = false;
    bool gyr_ret  = false;
    bool fifo_ret = false;

    if (true == a_accelerometer && true == this->accelerometer_config.enabled)
    {
        acc_ret = this->transmit.function(Registers::ctrl1_xl, 0, this->transmit.p_user_data);

        if (true == acc_ret && true == this->fifo_config.enabled)
        {
            if (true == this->gyroscope_config.enabled)
            {
                acc_ret = this->transmit.function(Registers::fifo_ctrl3, 0x8u, this->transmit.p_user_data);
            }
            else
            {
                acc_ret = this->transmit.function(Registers::fifo_ctrl3, 0x0u, this->transmit.p_user_data);
            }
        }

        if (true == acc_ret)
        {
            this->accelerometer_config.enabled          = false;
            this->accelerometer_config.fullscale_range  = Accelerometer_config::Fullscale_range::unknown;
            this->accelerometer_config.output_data_rate = Accelerometer_config::Output_data_rate::unknown;
        }
    }

    if (true == a_groscope && true == this->gyroscope_config.enabled)
    {
        gyr_ret = this->transmit.function(Registers::ctrl2_g, 0x0u, this->transmit.p_user_data);

        if (true == gyr_ret && true == this->fifo_config.enabled)
        {
            if (true == this->accelerometer_config.enabled)
            {
                gyr_ret = this->transmit.function(Registers::fifo_ctrl3, 0x1u, this->transmit.p_user_data);
            }
            else
            {
                gyr_ret = this->transmit.function(Registers::fifo_ctrl3, 0x0u, this->transmit.p_user_data);
            }
        }

        if (true == gyr_ret)
        {
            this->gyroscope_config.enabled          = false;
            this->gyroscope_config.fullscale_range  = Gyroscope_config::Fullscale_range::unknown;
            this->gyroscope_config.output_data_rate = Gyroscope_config::Output_data_rate::unknown;
        }
    }

    if (false == this->accelerometer_config.enabled && false == this->gyroscope_config.enabled &&
        true == this->fifo_config.enabled)
    {
        fifo_ret = this->transmit.function(Registers::fifo_ctrl5, 0x0u, this->transmit.p_user_data);

        if (true == fifo_ret)
        {
            this->fifo_config.enabled                  = false;
            this->fifo_config.pattern_repetition_count = 0;
        }
    }

    return (true == a_accelerometer ? acc_ret : true) && (true == a_groscope ? gyr_ret : true) &&
           (true == a_accelerometer && true == a_groscope ? fifo_ret : true);
}

bool LSM6DSL::get_axis_raw(Axis<int16_t>* a_p_gyr_data,
                           uint32_t a_gyr_data_buffer_capacity,
                           Axis<int16_t>* a_p_acc_data,
                           uint32_t a_acc_data_buffer_capacity)
{
    bool acc = this->accelerometer_config.enabled && nullptr != a_p_acc_data && 0 != a_acc_data_buffer_capacity;
    bool gyr = this->gyroscope_config.enabled && nullptr != a_p_gyr_data && 0 != a_gyr_data_buffer_capacity;

    if (true == this->fifo_config.enabled)
    {
        bool fifo_full = false;
        bool ret       = true;

        while (true == ret && false == fifo_full)
        {
            ret = this->is_fifo_full(&fifo_full);
        }

        for (uint32_t i = 0; i < this->fifo_config.pattern_repetition_count; i++)
        {
            uint32_t acc_samples_count = 0;
            uint32_t gyr_samples_count = 0;

            for (uint32_t j = 0;
                 j < (this->fifo_context.acc_samples_count + this->fifo_context.gyr_samples_count) && true == ret;
                 j++)
            {
                if (gyr_samples_count < this->fifo_context.gyr_samples_count)
                {
                    for (uint32_t si = 0; si < 3 && true == ret; si++)
                    {
                        int16_t sample = 0;
                        ret            = this->receive.function(Registers::fifo_data_out_l,
                                                     reinterpret_cast<uint8_t*>(&sample),
                                                     sizeof(sample),
                                                     this->receive.p_user_data);

                        if (gyr_samples_count < a_gyr_data_buffer_capacity)
                        {
                            a_p_gyr_data[i + gyr_samples_count].linear[si] = sample;
                        }
                    }

                    gyr_samples_count++;
                }

                if (acc_samples_count < this->fifo_context.acc_samples_count)
                {
                    for (uint32_t si = 0; si < 3 && true == ret; si++)
                    {
                        int16_t sample = 0;
                        ret            = this->receive.function(Registers::fifo_data_out_l,
                                                     reinterpret_cast<uint8_t*>(&(sample)),
                                                     sizeof(sample),
                                                     this->receive.p_user_data);

                        if (acc_samples_count < a_acc_data_buffer_capacity)
                        {
                            a_p_acc_data[i + acc_samples_count].linear[si] = sample;
                        }
                    }

                    acc_samples_count++;
                }
            }
        }

        if (true == ret)
        {
            ret = this->set_fifo_context(this->fifo_context);
        }

        return ret;
    }
    else
    {
        bool ret = true;

        if (true == gyr)
        {
            for (uint32_t i = 0; i < a_gyr_data_buffer_capacity && true == ret; i++)
            {
                bool data_avaliable = false;
                while (true == ret && false == data_avaliable)
                {
                    ret = this->is_gyroscope_data_avaliable(&data_avaliable);
                }

                ret = true == ret &&
                      this->get_axis(
                          Registers::outx_l_g, &(a_p_gyr_data[i].x), &(a_p_gyr_data[i].y), &(a_p_gyr_data[i].z));
            }
        }

        if (true == acc)
        {
            for (uint32_t i = 0; i < a_acc_data_buffer_capacity && true == ret; i++)
            {
                bool data_avaliable = false;
                while (true == ret && false == data_avaliable)
                {
                    ret = this->is_accelerometer_data_avaliable(&data_avaliable);
                }

                ret = true == ret &&
                      this->get_axis(
                          Registers::outx_l_xl, &(a_p_acc_data[i].x), &(a_p_acc_data[i].y), &(a_p_acc_data[i].z));
            }
        }

        return ret;
    }

    return false;
}

bool LSM6DSL::get_axis_scaled(Axis<float>* a_p_gyr_data,
                              uint32_t a_gyr_data_buffer_capacity,
                              Axis<float>* a_p_acc_data,
                              uint32_t a_acc_data_buffer_capacity)
{
    bool ret = false;
    bool acc = this->accelerometer_config.enabled && nullptr != a_p_acc_data && 0 != a_acc_data_buffer_capacity;
    bool gyr = this->gyroscope_config.enabled && nullptr != a_p_gyr_data && 0 != a_gyr_data_buffer_capacity;

    if (true == this->fifo_config.enabled)
    {
        bool fifo_full = false;

        do
        {
            ret = this->is_fifo_full(&fifo_full);
        } while (true == ret && false == fifo_full);

        float gyr_sensitivity =
            gyroscope_sensitivity_lut[static_cast<uint8_t>(this->gyroscope_config.fullscale_range) >> 1u];

        float acc_sensitivity =
            accelerometer_sensitivity_lut[static_cast<uint8_t>(this->accelerometer_config.fullscale_range) >> 2u];

        for (uint32_t i = 0; i < this->fifo_config.pattern_repetition_count; i++)
        {
            uint32_t acc_samples_count = 0;
            uint32_t gyr_samples_count = 0;

            for (uint32_t j = 0;
                 j < (this->fifo_context.acc_samples_count + this->fifo_context.gyr_samples_count) && true == ret;
                 j++)
            {
                if (gyr_samples_count < this->fifo_context.gyr_samples_count)
                {
                    for (uint32_t si = 0; si < 3 && true == ret; si++)
                    {
                        int16_t sample = 0;
                        ret            = this->receive.function(Registers::fifo_data_out_l,
                                                     reinterpret_cast<uint8_t*>(&sample),
                                                     sizeof(sample),
                                                     this->receive.p_user_data);

                        if (gyr_samples_count < a_gyr_data_buffer_capacity)
                        {
                            a_p_gyr_data[i + gyr_samples_count].linear[si] =
                                static_cast<float>(sample) * gyr_sensitivity;
                        }
                    }

                    gyr_samples_count++;
                }

                if (acc_samples_count < this->fifo_context.acc_samples_count)
                {
                    for (uint32_t si = 0; si < 3 && true == ret; si++)
                    {
                        int16_t sample = 0;
                        ret            = this->receive.function(Registers::fifo_data_out_l,
                                                     reinterpret_cast<uint8_t*>(&(sample)),
                                                     sizeof(sample),
                                                     this->receive.p_user_data);

                        if (acc_samples_count < a_acc_data_buffer_capacity)
                        {
                            a_p_acc_data[i + acc_samples_count].linear[si] =
                                static_cast<float>(sample) * acc_sensitivity;
                        }
                    }

                    acc_samples_count++;
                }
            }
        }

        if (true == ret)
        {
            ret = this->set_fifo_context(this->fifo_context);
        }
    }
    else
    {
        if (true == gyr)
        {
            int16_t sample[3] = { 0 };
            float sensitivity =
                gyroscope_sensitivity_lut[static_cast<uint8_t>(this->gyroscope_config.fullscale_range) >> 1u];

            for (uint32_t i = 0; i < a_gyr_data_buffer_capacity; i++)
            {
                bool data_avaliable = false;

                do
                {
                    ret = this->is_gyroscope_data_avaliable(&data_avaliable);
                } while (true == ret && false == data_avaliable);

                if (true == ret)
                {
                    ret = this->get_axis(Registers::outx_l_g, &(sample[0]), &(sample[1]), &(sample[2]));

                    if (true == ret)
                    {
                        a_p_gyr_data[i].x = static_cast<float>(sample[0]) * sensitivity;
                        a_p_gyr_data[i].y = static_cast<float>(sample[1]) * sensitivity;
                        a_p_gyr_data[i].z = static_cast<float>(sample[2]) * sensitivity;
                    }
                }
            }
        }

        if (true == acc)
        {
            int16_t sample[3] = { 0 };
            float sensitivity =
                accelerometer_sensitivity_lut[static_cast<uint8_t>(this->accelerometer_config.fullscale_range) >> 2u];

            for (uint32_t i = 0; i < a_acc_data_buffer_capacity; i++)
            {
                bool data_avaliable = false;

                do
                {
                    ret = this->is_accelerometer_data_avaliable(&data_avaliable);
                } while (true == ret && false == data_avaliable);

                if (true == ret)
                {
                    ret = this->get_axis(Registers::outx_l_xl, &(sample[0]), &(sample[1]), &(sample[2]));

                    if (true == ret)
                    {
                        a_p_acc_data[i].x = static_cast<float>(sample[0]) * sensitivity;
                        a_p_acc_data[i].y = static_cast<float>(sample[1]) * sensitivity;
                        a_p_acc_data[i].z = static_cast<float>(sample[2]) * sensitivity;
                    }
                }
            }
        }
    }

    return ret;
}

bool LSM6DSL::set_fifo_context(const Fifo_context& a_context)
{
    bool ret = this->transmit.function(Registers::fifo_ctrl5, 0, this->transmit.p_user_data);

    if (true == ret)
    {
        ret = this->transmit.function(Registers::fifo_ctrl5, 0x1u | a_context.odr << 3u, this->transmit.p_user_data);

        if (true == ret)
        {
            ret = this->transmit.function(Registers::fifo_ctrl3,
                                          decimation_to_flag(a_context.acc_decimation) |
                                              (decimation_to_flag(a_context.gyr_decimation) << 3u),
                                          this->transmit.p_user_data);
        }
    }

    return ret;
}

bool LSM6DSL::get_axis(uint8_t a_outx_register, int16_t* a_p_outx, int16_t* a_p_outy, int16_t* a_p_outz)
{
    return this->receive.function(a_outx_register + 0,
                                  reinterpret_cast<uint8_t*>(a_p_outx),
                                  sizeof(*a_p_outx),
                                  this->receive.p_user_data) &&
           this->receive.function(a_outx_register + 2,
                                  reinterpret_cast<uint8_t*>(a_p_outy),
                                  sizeof(*a_p_outy),
                                  this->receive.p_user_data) &&
           this->receive.function(
               a_outx_register + 4, reinterpret_cast<uint8_t*>(a_p_outz), sizeof(*a_p_outz), this->receive.p_user_data);
}

bool LSM6DSL::is_fifo_full(bool* a_p_status) const
{
    assert(nullptr != a_p_status);

    uint8_t v = 0;
    bool ret  = this->receive.function(Registers::fifo_status2, &v, sizeof(v), this->receive.p_user_data);

    if (true == ret)
    {
        (*a_p_status) = is_bit_on(v, 7);
    }

    return ret;
}

bool LSM6DSL::is_accelerometer_data_avaliable(bool* a_p_status) const
{
    assert(nullptr != a_p_status);

    uint8_t v = 0;
    bool ret  = this->receive.function(Registers::status_reg, &v, sizeof(v), this->transmit.p_user_data);

    if (true == ret)
    {
        *(a_p_status) = is_bit_on(v, 0);
    }

    return ret;
}

bool LSM6DSL::is_gyroscope_data_avaliable(bool* a_p_status) const
{
    assert(nullptr != a_p_status);

    uint8_t v = 0;
    bool ret  = this->receive.function(Registers::status_reg, &v, sizeof(v), this->transmit.p_user_data);

    if (true == ret)
    {
        *(a_p_status) = is_bit_on(v, 1);
    }

    return ret;
}

bool LSM6DSL::reset()
{
    uint8_t v = 0;
    bool ret  = this->receive.function(Registers::ctrl3_c, &v, sizeof(v), this->receive.p_user_data);

    if (true == ret)
    {
        set_bit(&v, 0);
        ret = this->transmit.function(Registers::ctrl3_c, v, this->transmit.p_user_data);
    }

    return ret;
}

bool LSM6DSL::is_alive() const
{
    uint8_t v = 0;
    if (true == this->receive.function(0xFu, &v, sizeof(v), this->receive.p_user_data))
    {
        return 0x6Au == v;
    }

    return false;
}

} // namespace devices
} // namespace cml