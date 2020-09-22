#lib
C_SOURCE_PATHS   := $(C_SOURCE_PATHS)   $(CML_ROOT)/lib/
CPP_SOURCE_PATHS := $(CPP_SOURCE_PATHS) $(CML_ROOT)/lib/         \
                    $(CML_ROOT)/lib/cml/hal/                     \
                    $(CML_ROOT)/lib/cml/utils/                   \
                    $(CML_ROOT)/lib/cml/common/                  \
                    $(CML_ROOT)/lib/cml/debug/                   \
                    $(CML_ROOT)/lib/cml/hal/stm32l011xx/         \
                    $(CML_ROOT)/lib/cml/hal/core/                \
                    $(CML_ROOT)/lib/soc/                         \
                    $(CML_ROOT)/lib/soc/stm32l011xx/             \
                    $(CML_ROOT)/lib/soc/stm32l011xx/system/      \
                    $(CML_ROOT)/lib/soc/stm32l011xx/peripherals/

INCLUDE_PATH     := $(INCLUDE_PATH) $(CML_ROOT)/lib/

#sample
C_SOURCE_PATHS   := $(C_SOURCE_PATHS)   $(ROOT)/
CPP_SOURCE_PATHS := $(CPP_SOURCE_PATHS) $(ROOT)/
INCLUDE_PATH     := $(INCLUDE_PATH)     $(ROOT)/

#cmsis/include
INCLUDE_PATH := $(INCLUDE_PATH) $(CML_ROOT)/externals/CMSIS/Include/

#cmsis/device/st/stm32l0xx
C_SOURCE_PATHS := $(C_SOURCE_PATHS) $(CML_ROOT)/externals/CMSIS/Device/ST/STM32L0xx
S_SOURCE_PATHS := $(S_SOURCE_PATHS) $(CML_ROOT)/externals/CMSIS/Device/ST/STM32L0xx
INCLUDE_PATH   := $(INCLUDE_PATH)   $(CML_ROOT)/externals/CMSIS/Device/ST/STM32L0xx