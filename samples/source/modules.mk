#
#  Name: modules.mk
#
#  Copyright (c) Mateusz Semegen and contributors. All rights reserved.
#  Licensed under the MIT license. See LICENSE file in the project root for details.
#

INCLUDE_PATH     := $(ROOT)/                                                \
                    $(CML_ROOT)/lib/                                        \
                    $(CML_ROOT)/externals/CMSIS/Include/                    \
                    $(CML_ROOT)/externals/CMSIS/Device/ST/STM32L4xx/Include/

C_SOURCE_PATHS   := $(CML_ROOT)/lib                                 \
                    $(ROOT)/..                                      \
                    $(CML_ROOT)/externals/CMSIS/Device/ST/STM32L4xx

CPP_SOURCE_PATHS := $(CPP_SOURCE_PATHS)                           \
                    $(CML_ROOT)/lib/cml/hal                       \
                    $(CML_ROOT)/lib/cml/utils                     \
                    $(CML_ROOT)/lib/cml/debug                     \
                    $(CML_ROOT)/lib/soc                           \
                    $(CML_ROOT)/lib/soc/m4                        \
                    $(CML_ROOT)/lib/soc/m4/stm32l4                \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/ADC            \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/Basic_timer    \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/CRC32          \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/GPIO           \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/GPIO/bsp       \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/I2C            \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/I2C/bsp        \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/internal_flash \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/IWDG           \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/mcu            \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/pwr            \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/RNG            \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/RS485          \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/SPI            \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/SPI/bsp        \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/USART          \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/USART/bsp      \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/WWDG

STARTUP_FILE := $(CML_ROOT)/externals/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l452xx.s