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

C_SOURCE_PATHS   := $(C_SOURCE_PATHS)                               \
                    $(CML_ROOT)/lib/                                \
                    $(ROOT)/../                                     \
                    $(CML_ROOT)/externals/CMSIS/Device/ST/STM32L4xx/

CPP_SOURCE_PATHS := $(ROOT)/                                   \
                    $(CML_ROOT)/lib/cml                        \
                    $(CML_ROOT)/lib/cml/hal/                   \
                    $(CML_ROOT)/lib/cml/utils/                 \
                    $(CML_ROOT)/lib/cml/debug/                 \
                    $(CML_ROOT)/lib/soc/                       \
                    $(CML_ROOT)/lib/soc/m4/                    \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/            \
                    $(CML_ROOT)/lib/soc/m4/stm32l4/peripherals/

STARTUP_FILE := $(CML_ROOT)/externals/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l452xx.s