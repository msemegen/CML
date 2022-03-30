#pragma once

/*
 *   Name: defs.hpp
 *
 *   Copyright (c) Mateusz Semegen and contributors. All rights reserved.
 *   Licensed under the MIT license. See LICENSE file in the project root for details.
 */

#if defined(STM32L431CBT) || defined(STM32L431CCT) || defined(STM32L431CBU) || defined(STM32L431CCU) ||     \
    defined(STM32L431CBY) || defined(STM32L431CCY) || defined(STM32L431KBU) || defined(STM32L431KCU) ||     \
    defined(STM32L431RBI) || defined(STM32L431RCI) || defined(STM32L431RBT) || defined(STM32L431RCT) ||     \
    defined(STM32L431RBY) || defined(STM32L431RCY) || defined(STM32L431VCI) || defined(STM32L431VCT) ||     \
    defined(STM32L432KBU) || defined(STM32L432KCU) || defined(STM32L433CBT) || defined(STM32L433CCT) ||     \
    defined(STM32L433CBU) || defined(STM32L433CCU) || defined(STM32L433CBY) || defined(STM32L433CCY) ||     \
    defined(STM32L433RBI) || defined(STM32L433RCI) || defined(STM32L433RBT) || defined(STM32L433RCT) ||     \
    defined(STM32L433RBY) || defined(STM32L433RCY) || defined(STM32L433RCTxP) || defined(STM32L433VCI) ||   \
    defined(STM32L433VCT) || defined(STM32L442KCU) || defined(STM32L443CCF) || defined(STM32L443CCT) ||     \
    defined(STM32L443CCU) || defined(STM32L443CCY) || defined(STM32L443RCI) || defined(STM32L443RCT) ||     \
    defined(STM32L443RCY) || defined(STM32L443VCI) || defined(STM32L443VCT) || defined(STM32L451CCU) ||     \
    defined(STM32L451CEU) || defined(STM32L451CET) || defined(STM32L451RCI) || defined(STM32L451REI) ||     \
    defined(STM32L451RCT) || defined(STM32L451RET) || defined(STM32L451RCY) || defined(STM32L451REY) ||     \
    defined(STM32L451VCI) || defined(STM32L451VEI) || defined(STM32L451VCT) || defined(STM32L451VET) ||     \
    defined(STM32L452CCU) || defined(STM32L452CEU) || defined(STM32L452CET) || defined(STM32L452CETxP) ||   \
    defined(STM32L452RCI) || defined(STM32L452REI) || defined(STM32L452RCT) || defined(STM32L452RET) ||     \
    defined(STM32L452RCY) || defined(STM32L452REY) || defined(STM32L452RETxP) || defined(STM32L452REYxP) || \
    defined(STM32L452VCI) || defined(STM32L452VEI) || defined(STM32L452VCT) || defined(STM32L452VET) ||     \
    defined(STM32L462CET) || defined(STM32L462CEU) || defined(STM32L462REI) || defined(STM32L462RET) ||     \
    defined(STM32L462REY) || defined(STM32L462VEI) || defined(STM32L462VET)
#define SOC_ADC1_PRESENT
#define SOC_PLLSAI_PRESENT
#define SOC_PLL_P_PRESENT
#endif

#if defined(STM32L412C8T) || defined(STM32L412C8U) || defined(STM32L412CBT) || defined(STM32L412CBTxP) ||   \
    defined(STM32L412CBU) || defined(STM32L412CBUxP) || defined(STM32L412K8T) || defined(STM32L412K8U) ||   \
    defined(STM32L412KBT) || defined(STM32L412KBU) || defined(STM32L412R8I) || defined(STM32L412R8T) ||     \
    defined(STM32L412RBI) || defined(STM32L412RBIxP) || defined(STM32L412RBT) || defined(STM32L412RBTxP) || \
    defined(STM32L412T8Y) || defined(STM32L412TBY) || defined(STM32L412TBYxP) || defined(STM32L422CBT) ||   \
    defined(STM32L422CBU) || defined(STM32L422KBT) || defined(STM32L422KBU) || defined(STM32L422RBI) ||     \
    defined(STM32L422RBT) || defined(STM32L422TBY)
#define SOC_ADC1_PRESENT
#define SOC_ADC2_PRESENT
#endif

#if defined(STM32L412C8T) || defined(STM32L412C8U) || defined(STM32L412CBT) || defined(STM32L412CBTxP) || \
    defined(STM32L412CBU) || defined(STM32L412CBUxP) || defined(STM32L412K8T) || defined(STM32L412K8U) || \
    defined(STM32L412KBT) || defined(STM32L412KBU) || defined(STM32L412T8Y) || defined(STM32L412TBY) ||   \
    defined(STM32L412TBYxP) || defined(STM32L422CBT) || defined(STM32L422CBU) || defined(STM32L422KBT) || \
    defined(STM32L422KBU) || defined(STM32L422TBY) || defined(STM32L431CBT) || defined(STM32L431CCT) ||   \
    defined(STM32L431CBU) || defined(STM32L431CCU) || defined(STM32L431CBY) || defined(STM32L431CCY) ||   \
    defined(STM32L431KBU) || defined(STM32L431KCU) || defined(STM32L432KBU) || defined(STM32L432KCU) ||   \
    defined(STM32L433CBT) || defined(STM32L433CCT) || defined(STM32L433CBU) || defined(STM32L433CCU) ||   \
    defined(STM32L442KCU) || defined(STM32L443CCT) || defined(STM32L443CCU) || defined(STM32L451CCU) ||   \
    defined(STM32L451CEU) || defined(STM32L451CET) || defined(STM32L452CCU) || defined(STM32L452CEU) ||   \
    defined(STM32L452CET) || defined(STM32L452CETxP) || defined(STM32L462CET) || defined(STM32L462CEU)
#define SOC_ADC_CHANNELS_5_6_7_8_9_10_11_12_15_16
#endif
#if defined(STM32L412R8I) || defined(STM32L412R8T) || defined(STM32L412RBI) || defined(STM32L412RBT) ||   \
    defined(STM32L422RBI) || defined(STM32L422RBT) || defined(STM32L431RBI) || defined(STM32L431RCI) ||   \
    defined(STM32L431RBT) || defined(STM32L431RCT) || defined(STM32L431RBY) || defined(STM32L431RCY) ||   \
    defined(STM32L431VCI) || defined(STM32L431VCT) || defined(STM32L433RBI) || defined(STM32L433RCI) ||   \
    defined(STM32L433RBT) || defined(STM32L433RCT) || defined(STM32L433RBY) || defined(STM32L433RCY) ||   \
    defined(STM32L433VCI) || defined(STM32L433VCT) || defined(STM32L443RCI) || defined(STM32L443RCT) ||   \
    defined(STM32L443RCY) || defined(STM32L443VCI) || defined(STM32L443VCT) || defined(STM32L451RCI) ||   \
    defined(STM32L451REI) || defined(STM32L451RCT) || defined(STM32L451RET) || defined(STM32L451RCY) ||   \
    defined(STM32L451REY) || defined(STM32L451VCI) || defined(STM32L451VEI) || defined(STM32L451VCT) ||   \
    defined(STM32L451VET) || defined(STM32L452RCI) || defined(STM32L452REI) || defined(STM32L452RCT) ||   \
    defined(STM32L452RET) || defined(STM32L452RCY) || defined(STM32L452REY) || defined(STM32L452REYxP) || \
    defined(STM32L452VCI) || defined(STM32L452VEI) || defined(STM32L452VCT) || defined(STM32L452VET) ||   \
    defined(STM32L462REI) || defined(STM32L462RET) || defined(STM32L462REY) || defined(STM32L462VEI) ||   \
    defined(STM32L462VET)
#define SOC_ADC_CHANNELS_1_2_3_4_5_6_7_8_9_10_11_12_13_14_15_16
#endif

#if defined(STM32L412RBIxP) || defined(STM32L412RBTxP) || defined(STM32L433RCTxP) || defined(STM32L452RETxP)
#define SOC_ADC_CHANNELS_1_2_3_4_5_6_7_8_9_10_11_12_13_15_16
#endif

#if defined(STM32L433CBY) || defined(STM32L433CCY) || defined(STM32L443CCF) || defined(STM32L443CCY)
#define SOC_ADC_CHANNELS_4_5_6_7_8_9_10_11_12_15_16
#endif

#if defined(STM32L412K8T) || defined(STM32L412K8U) || defined(STM32L412KBT) || defined(STM32L412KBU) ||   \
    defined(STM32L412T8Y) || defined(STM32L412TBY) || defined(STM32L412TBYxP) || defined(STM32L422KBT) || \
    defined(STM32L422KBU) || defined(STM32L422TBY) || defined(STM32L431KBU) || defined(STM32L431KCU) ||   \
    defined(STM32L432KBU) || defined(STM32L432KCU) || defined(STM32L442KCU)
#define SOC_USART1_PRESENT
#define SOC_USART2_PRESENT
#endif

#if defined(STM32L412C8T) || defined(STM32L412C8U) || defined(STM32L412CBT) || defined(STM32L412CBTxP) ||   \
    defined(STM32L412CBU) || defined(STM32L412CBUxP) || defined(STM32L412R8I) || defined(STM32L412R8T) ||   \
    defined(STM32L412RBI) || defined(STM32L412RBIxP) || defined(STM32L412RBT) || defined(STM32L412RBTxP) || \
    defined(STM32L422CBT) || defined(STM32L422CBU) || defined(STM32L422RBI) || defined(STM32L422RBT) ||     \
    defined(STM32L431CBT) || defined(STM32L431CCT) || defined(STM32L431CBU) || defined(STM32L431CCU) ||     \
    defined(STM32L431CBY) || defined(STM32L431CCY) || defined(STM32L431RBI) || defined(STM32L431RCI) ||     \
    defined(STM32L431RBT) || defined(STM32L431RCT) || defined(STM32L431RBY) || defined(STM32L431RCY) ||     \
    defined(STM32L431VCI) || defined(STM32L431VCT) || defined(STM32L433CBT) || defined(STM32L433CCT) ||     \
    defined(STM32L433CBU) || defined(STM32L433CCU) || defined(STM32L433CBY) || defined(STM32L433CCY) ||     \
    defined(STM32L433RBI) || defined(STM32L433RCI) || defined(STM32L433RBT) || defined(STM32L433RCT) ||     \
    defined(STM32L433RBY) || defined(STM32L433RCY) || defined(STM32L433RCTxP) || defined(STM32L433VCI) ||   \
    defined(STM32L433VCT) || defined(STM32L443CCF) || defined(STM32L443CCT) || defined(STM32L443CCU) ||     \
    defined(STM32L443CCY) || defined(STM32L443RCI) || defined(STM32L443RCT) || defined(STM32L443RCY) ||     \
    defined(STM32L443VCI) || defined(STM32L443VCT) || defined(STM32L451CCU) || defined(STM32L451CEU) ||     \
    defined(STM32L451CET) || defined(STM32L451RCI) || defined(STM32L451REI) || defined(STM32L451RCT) ||     \
    defined(STM32L451RET) || defined(STM32L451RCY) || defined(STM32L451REY) || defined(STM32L451VCI) ||     \
    defined(STM32L451VEI) || defined(STM32L451VCT) || defined(STM32L451VET) || defined(STM32L452CCU) ||     \
    defined(STM32L452CEU) || defined(STM32L452CET) || defined(STM32L452CETxP) || defined(STM32L452RCI) ||   \
    defined(STM32L452REI) || defined(STM32L452RCT) || defined(STM32L452RET) || defined(STM32L452RCY) ||     \
    defined(STM32L452REY) || defined(STM32L452RETxP) || defined(STM32L452REYxP) || defined(STM32L452VCI) || \
    defined(STM32L452VEI) || defined(STM32L452VCT) || defined(STM32L452VET) || defined(STM32L462CET) ||     \
    defined(STM32L462CEU) || defined(STM32L462REI) || defined(STM32L462RET) || defined(STM32L462REY) ||     \
    defined(STM32L462VEI) || defined(STM32L462VET)
#define SOC_USART1_PRESENT
#define SOC_USART2_PRESENT
#define SOC_USART3_PRESENT
#define SOC_I2C2_PRESENT
#define SOC_SPI2_PRESENT
#endif

#if defined(STM32L412C8T) || defined(STM32L412C8U) || defined(STM32L412CBT) || defined(STM32L412CBTxP) ||   \
    defined(STM32L412CBU) || defined(STM32L412CBUxP) || defined(STM32L412K8T) || defined(STM32L412K8U) ||   \
    defined(STM32L412KBT) || defined(STM32L412KBU) || defined(STM32L412R8I) || defined(STM32L412R8T) ||     \
    defined(STM32L412RBI) || defined(STM32L412RBIxP) || defined(STM32L412RBT) || defined(STM32L412RBTxP) || \
    defined(STM32L412T8Y) || defined(STM32L412TBY) || defined(STM32L412TBYxP) || defined(STM32L422CBT) ||   \
    defined(STM32L422CBU) || defined(STM32L422KBT) || defined(STM32L422KBU) || defined(STM32L422RBI) ||     \
    defined(STM32L422RBT) || defined(STM32L422TBY) || defined(STM32L431CBT) || defined(STM32L431CCT) ||     \
    defined(STM32L431CBU) || defined(STM32L431CCU) || defined(STM32L431CBY) || defined(STM32L431CCY) ||     \
    defined(STM32L431KBU) || defined(STM32L431KCU) || defined(STM32L431RBI) || defined(STM32L431RCI) ||     \
    defined(STM32L431RBT) || defined(STM32L431RCT) || defined(STM32L431RBY) || defined(STM32L431RCY) ||     \
    defined(STM32L431VCI) || defined(STM32L431VCT) || defined(STM32L432KBU) || defined(STM32L432KCU) ||     \
    defined(STM32L433CBT) || defined(STM32L433CCT) || defined(STM32L433CBU) || defined(STM32L433CCU) ||     \
    defined(STM32L433CBY) || defined(STM32L433CCY) || defined(STM32L433RBI) || defined(STM32L433RCI) ||     \
    defined(STM32L433RBT) || defined(STM32L433RCT) || defined(STM32L433RBY) || defined(STM32L433RCY) ||     \
    defined(STM32L433RCTxP) || defined(STM32L433VCI) || defined(STM32L433VCT) || defined(STM32L442KCU) ||   \
    defined(STM32L443CCF) || defined(STM32L443CCT) || defined(STM32L443CCU) || defined(STM32L443CCY) ||     \
    defined(STM32L443RCI) || defined(STM32L443RCT) || defined(STM32L443RCY) || defined(STM32L443VCI) ||     \
    defined(STM32L443VCT) || defined(STM32L451CCU) || defined(STM32L451CEU) || defined(STM32L451CET) ||     \
    defined(STM32L451RCI) || defined(STM32L451REI) || defined(STM32L451RCT) || defined(STM32L451RET) ||     \
    defined(STM32L451RCY) || defined(STM32L451REY) || defined(STM32L451VCI) || defined(STM32L451VEI) ||     \
    defined(STM32L451VCT) || defined(STM32L451VET) || defined(STM32L452CCU) || defined(STM32L452CEU) ||     \
    defined(STM32L452CET) || defined(STM32L452CETxP) || defined(STM32L452RCI) || defined(STM32L452REI) ||   \
    defined(STM32L452RCT) || defined(STM32L452RET) || defined(STM32L452RCY) || defined(STM32L452REY) ||     \
    defined(STM32L452RETxP) || defined(STM32L452REYxP) || defined(STM32L452VCI) || defined(STM32L452VEI) || \
    defined(STM32L452VCT) || defined(STM32L452VET) || defined(STM32L462CET) || defined(STM32L462CEU) ||     \
    defined(STM32L462REI) || defined(STM32L462RET) || defined(STM32L462REY) || defined(STM32L462VEI) ||     \
    defined(STM32L462VET)
#define SOC_I2C1_PRESENT
#define SOC_I2C3_PRESENT
#define SOC_SPI1_PRESENT
#endif

#if defined(STM32L451CCU) || defined(STM32L451CEU) || defined(STM32L451CET) || defined(STM32L451RCI) ||   \
    defined(STM32L451REI) || defined(STM32L451RCT) || defined(STM32L451RET) || defined(STM32L451RCY) ||   \
    defined(STM32L451REY) || defined(STM32L451VCI) || defined(STM32L451VEI) || defined(STM32L451VCT) ||   \
    defined(STM32L451VET) || defined(STM32L452CCU) || defined(STM32L452CEU) || defined(STM32L452CET) ||   \
    defined(STM32L452CETxP) || defined(STM32L452RCI) || defined(STM32L452REI) || defined(STM32L452RCT) || \
    defined(STM32L452RET) || defined(STM32L452RCY) || defined(STM32L452REY) || defined(STM32L452RETxP) || \
    defined(STM32L452REYxP) || defined(STM32L452VCI) || defined(STM32L452VEI) || defined(STM32L452VCT) || \
    defined(STM32L452VET) || defined(STM32L462CET) || defined(STM32L462CEU) || defined(STM32L462REI) ||   \
    defined(STM32L462RET) || defined(STM32L462REY) || defined(STM32L462VEI) || defined(STM32L462VET)
#define SOC_I2C4_PRESENT
#endif

#if defined(STM32L431CBT) || defined(STM32L431CCT) || defined(STM32L431CBU) || defined(STM32L431CCU) ||       \
    defined(STM32L431CBY) || defined(STM32L431CCY) || defined(STM32L431KBU) || defined(STM32L431KCU) ||       \
    defined(STM32L431RBI) || defined(STM32L431RCI) || defined(STM32L431RBT) || defined(STM32L431RCT) ||       \
    defined(STM32L431RBY) || defined(STM32L431RCY) || defined(STM32L431VCI) || defined(STM32L431VCT) ||       \
    defined(STM32L432KBU) || defined(STM32L432KCU) || defined(STM32L433CBT) || defined(STM32L433CCT) ||       \
    defined(STM32L433CBU) || defined(STM32L433CCU) || defined(STM32L433CBY) || defined(STM32L433CCY) ||       \
    defined(STM32L433RBI) || defined(STM32L433RCI) || defined(STM32L433RBT) || defined(STM32L433RCT) ||       \
    defined(STM32L433RBY) || defined(STM32L433RCY) || defined(STM32L433RCTxP) || defined(STM32L433VCI) ||     \
    defined(STM32L433VCT) || defined(STM32L442KCU) || defined(STM32L443CCF) || defined(STM32L443CCT) ||       \
    defined(STM32L443CCU) || defined(STM32L443CCY) || defined(STM32L443RCI) || defined(STM32L443RCT) ||       \
    defined(STM32L443RCY) || defined(STM32L443VCI) || defined(STM32L443VCT) || defined(STM32L451CCU) ||       \
    defined(STM32L451CEU) || defined(STM32L451CET) || defined(STM32L451RCI) || defined(STM32L451REI) ||       \
    defined(STM32L451RCT) || defined(STM32L451RET) || defined(STM32L451RCY) || defined(STM32L451REY) ||       \
    defined(STM32L451VCI) || defined(STM32L451VEI) || defined(STM32L451VCT) || defined(STM32L451VET) ||       \
    defined(STM32L452CCU) || defined(STM32L452CEU) || defined(STM32L452CET) || defined(STM32L452CETxP) ||     \
    defined(STM32L452RCI) || defined(STM32L452REI) || defined(STM32L452RCT) || defined(STM32L452RET) ||       \
    defined(STM32L452RCY) || defined(STM32L452REY) || defined(STM32L452RETxP) || defined(STM32L452REYxP) ||   \
    defined(STM32L452VCI) || defined(STM32L452VEI) || defined(STM32L452VCT) || defined(STM32L452VET) ||       \
    defined(STM32L462CET) || defined(STM32L462CEU) || defined(STM32L462REI) || defined(STM32L462RET) ||       \
    defined(STM32L462REY) || defined(STM32L462VEI) || defined(STM32L462VET)
#define SOC_SPI3_PRESENT
#endif