/*
    Name: main.cpp

    Copyright(c) 2019 Mateusz Semegen
    This code is licensed under MIT license (see LICENSE file for details)
*/

//cml
#include <hal/ADC.hpp>
#include <hal/GPIO.hpp>
#include <hal/mcu.hpp>
#include <hal/systick.hpp>
#include <common/bit.hpp>
#include <common/frequency.hpp>

#include <hal/USART.hpp>
#include <utils/Console.hpp>
#include <utils/sleep.hpp>

volatile bool wait = true;
volatile cml::common::uint16 d = 0;

bool callback(cml::common::uint16 a_data, bool a_series_emd, bool a_timeout)
{
    wait = false;
    d = a_data;
    return true;
}

#define VREFINT_CAL_ADDR                   ((uint16_t*) (0x1FFF75AAU)) /* Internal voltage reference, address of parameter VREFINT_CAL: VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC), Vref+ = 3.0 V (tolerance: +-10 mV). */
#define VREFINT_CAL_VREF                   ( 3000U)                    /* Analog voltage reference (Vref+) value with which temperature sensor has been calibrated in production (tolerance: +-10 mV) (unit: mV). */
/* Temperature sensor */
#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) (0x1FFF75A8U)) /* Internal temperature sensor, address of parameter TS_CAL1: On STM32L4, temperature sensor ADC raw data acquired at temperature  30 DegC (tolerance: +-5 DegC), Vref+ = 3.0 V (tolerance: +-10 mV). */
#define TEMPSENSOR_CAL2_ADDR               ((uint16_t*) (0x1FFF75CAU)) /* Internal temperature sensor, address of parameter TS_CAL2: On STM32L4, temperature sensor ADC raw data acquired at temperature 110 DegC (tolerance: +-5 DegC), Vref+ = 3.0 V (tolerance: +-10 mV). */
#define TEMPSENSOR_CAL1_TEMP               (( int32_t)   30)           /* Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL1_ADDR (tolerance: +-5 DegC) (unit: DegC). */
#define TEMPSENSOR_CAL2_TEMP               (( int32_t)  110)           /* Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL2_ADDR (tolerance: +-5 DegC) (unit: DegC). */
#define TEMPSENSOR_CAL_VREFANALOG          ( 3000U)                    /* Analog voltage reference (Vref+) voltage with which temperature sensor has been calibrated in production (+-10 mV) (unit: mV). */

/* ADC registers bits positions */
#define ADC_CFGR_RES_BITOFFSET_POS         ( 3U) /* Value equivalent to POSITION_VAL(ADC_CFGR_RES) */
#define ADC_CFGR_AWD1SGL_BITOFFSET_POS     (22U) /* Value equivalent to POSITION_VAL(ADC_CFGR_AWD1SGL) */
#define ADC_CFGR_AWD1EN_BITOFFSET_POS      (23U) /* Value equivalent to POSITION_VAL(ADC_CFGR_AWD1EN) */
#define ADC_CFGR_JAWD1EN_BITOFFSET_POS     (24U) /* Value equivalent to POSITION_VAL(ADC_CFGR_JAWD1EN) */
#define ADC_TR1_HT1_BITOFFSET_POS          (16U) /* Value equivalent to POSITION_VAL(ADC_TR1_HT1) */


#define __LL_ADC_CONVERT_DATA_RESOLUTION(__DATA__,\
                                         __ADC_RESOLUTION_CURRENT__,\
                                         __ADC_RESOLUTION_TARGET__)            \
  (((__DATA__)                                                                 \
    << ((__ADC_RESOLUTION_CURRENT__) >> (ADC_CFGR_RES_BITOFFSET_POS - 1U)))    \
   >> ((__ADC_RESOLUTION_TARGET__) >> (ADC_CFGR_RES_BITOFFSET_POS - 1U))       \
  )


#define __LL_ADC_CALC_TEMPERATURE(__VREFANALOG_VOLTAGE__,\
                                  __TEMPSENSOR_ADC_DATA__,\
                                  __ADC_RESOLUTION__)                              \
  (((( ((int32_t)((__LL_ADC_CONVERT_DATA_RESOLUTION((__TEMPSENSOR_ADC_DATA__),     \
                                                    (__ADC_RESOLUTION__),          \
                                                    0x0)         \
                   * (__VREFANALOG_VOLTAGE__))                                     \
                  / TEMPSENSOR_CAL_VREFANALOG)                                     \
        - (int32_t) *TEMPSENSOR_CAL1_ADDR)                                         \
     ) * (int32_t)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)                    \
    ) / (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) \
   ) + TEMPSENSOR_CAL1_TEMP                                                        \
  )


int main()
{
    using namespace cml::common;
    using namespace cml::hal;

    mcu::enable_hsi_clock(mcu::Hsi_frequency::_16_MHz);
    mcu::set_sysclk(mcu::Sysclk_source::hsi, { mcu::Bus_prescalers::AHB::_1,
                                               mcu::Bus_prescalers::APB1::_1,
                                               mcu::Bus_prescalers::APB2::_1 });

    if (mcu::Sysclk_source::hsi == mcu::get_sysclk_source())
    {
        mcu::set_nvic({ mcu::NVIC_config::Grouping::_4, 16u << 4u });

        systick::enable(0x0);

        ADC adc_1(ADC::Id::_1);

        mcu::enable_dwt();

        GPIO gpio_c(GPIO::Id::c);
        gpio_c.enable();

        bool ok = adc_1.enable(ADC::Resolution::_12_bit, { ADC::Synchronous_clock::Source::pclk,
                                                           ADC::Synchronous_clock::Divider::_1 }, 20);

        if (true == ok)
        {
            Analog_pin channel_2(&gpio_c, 1);
            channel_2.enable(Input_pin::Pull::up);

            ADC::Channel channels[] = { ADC::Channel::Id::temperature_sensor, ADC::Channel::Sampling_time::_640_5_clock_cycles };

            adc_1.set_active_channels(channels, 1);

            uint16 buff = 0;

            while (true)
            {
                wait = true;
                adc_1.read_polling(&buff, 1);
                //while (true == wait);

                auto c1 = adc_1.get_temperature_sensor_calibration_data_1().data;
                auto c2 = adc_1.get_temperature_sensor_calibration_data_2().data;

                auto c11 = (int32_t)*TEMPSENSOR_CAL1_ADDR;
                auto c22 = (int32_t)*TEMPSENSOR_CAL2_ADDR;

                //volatile auto temp = (100.0f / (c2 - c1)) * (d - c1) + 30.0f;
                //temp = temp;
                //d = d >> 4;
                volatile auto temp = __LL_ADC_CALC_TEMPERATURE(3300, buff, 0x0);
                temp = temp;
                cml::utils::sleep::ms(10);

                //ADC_CCR_TSEN
                //uint16 data = 0;
                //adc_1.read_it()
                //adc_1.read_polling(&data, 1);
                //data = data;
            }
        }
    }


    while (true);
}