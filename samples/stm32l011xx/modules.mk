#lib
C_SOURCE_PATHS   := $(C_SOURCE_PATHS)   $(ROOT)/../../../lib/
CPP_SOURCE_PATHS := $(CPP_SOURCE_PATHS) $(ROOT)/../../../lib/ \
                    $(ROOT)/../../../lib/hal/                 \
                    $(ROOT)/../../../lib/utils/               \
                    $(ROOT)/../../../lib/common/              \
                    $(ROOT)/../../../lib/debug/               \
                    $(ROOT)/../../../lib/hal/stm32l011xx/     \
                    $(ROOT)/../../../lib/hal/core/


INCLUDE_PATH     := $(INCLUDE_PATH)     $(ROOT)/../../../lib/

#sample
C_SOURCE_PATHS   := $(C_SOURCE_PATHS)   $(ROOT)/ $(ROOT)/../
CPP_SOURCE_PATHS := $(CPP_SOURCE_PATHS) $(ROOT)/
INCLUDE_PATH     := $(INCLUDE_PATH)     $(ROOT)/

#cmsis/include
INCLUDE_PATH := $(INCLUDE_PATH) $(ROOT)/../../../externals/CMSIS/Include/

#cmsis/device/st/stm32l0xx
C_SOURCE_PATHS := $(C_SOURCE_PATHS) $(ROOT)/../../../externals/CMSIS/Device/ST/STM32L0xx
S_SOURCE_PATHS := $(S_SOURCE_PATHS) $(ROOT)/../../../externals/CMSIS/Device/ST/STM32L0xx
INCLUDE_PATH   := $(INCLUDE_PATH)   $(ROOT)/../../../externals/CMSIS/Device/ST/STM32L0xx