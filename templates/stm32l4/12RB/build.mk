#
#  Name: build.mk
#
#  Copyright (c) Mateusz Semegen and contributors. All rights reserved.
#  Licensed under the MIT license. See LICENSE file in the project root for details.
#

INCLUDE_PATH := $(INCLUDE_PATH) $(ROOT)

CFLAGS := $(addprefix -I, $(INCLUDE_PATH)) --specs=nano.specs
CFLAGS += -Wall -Wno-strict-aliasing -I. -c -fno-common -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffast-math -ffunction-sections -fdata-sections -fsingle-precision-constant
CFLAGS += -D$(MCU_FAMILY) -D$(MCU_TYPE) -DARM_MATH_CM4 -DARM_MATH_ROUNDING -D__FPU_PRESENT

CPPFLAGS := $(CFLAGS)
CPPFLAGS += -std=c++17 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics

C_SOURCE_FILES    := $(wildcard $(addsuffix /*.c, $(C_SOURCE_PATHS)))
C_OBJECTS_RELEASE := $(addprefix $(OUTDIR_RELEASE)/, $(notdir $(patsubst %.c, %_c.o,$(C_SOURCE_FILES))))
C_OBJECTS_DEBUG   := $(addprefix $(OUTDIR_DEBUG)/, $(notdir $(patsubst %.c, %_c.o,$(C_SOURCE_FILES))))

CPP_SOURCE_FILES    := $(wildcard $(addsuffix /*.cpp, $(CPP_SOURCE_PATHS)))
CPP_OBJECTS_RELEASE := $(addprefix $(OUTDIR_RELEASE)/, $(notdir $(patsubst %.cpp, %_cpp.o,$(CPP_SOURCE_FILES))))
CPP_OBJECTS_DEBUG   := $(addprefix $(OUTDIR_DEBUG)/, $(notdir $(patsubst %.cpp, %_cpp.o,$(CPP_SOURCE_FILES))))

STARTUP_DEBUG   := $(addprefix $(OUTDIR_DEBUG)/, $(notdir $(patsubst %.s, %.o,$(STARTUP_FILE))))
STARTUP_RELEASE := $(addprefix $(OUTDIR_RELEASE)/, $(notdir $(patsubst %.s, %.o,$(STARTUP_FILE))))

MAIN_LDFLAGS_COMMON  = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostartfiles --specs=nano.specs
MAIN_LDFLAGS_COMMON  +=-T$(LD_PATH)/STM32L412RBTxP_FLASH.ld -lgcc -fno-exceptions -fno-rtti -Wl,--gc-sections
MAIN_LDFLAGS_RELEASE = $(MAIN_LDFLAGS_COMMON) -Wl,-Map=$(OUTDIR)/$(OUTPUT_NAME).map,-cref
MAIN_LDFLAGS_DEBUG   = $(MAIN_LDFLAGS_COMMON) -Wl,-Map=$(OUTDIR)/$(OUTPUT_NAME)_d.map,-cref

ODFLAGS = -S
CPFLAGS = -Obinary

DEBUG_FLAGS = -O0 -g3 -DCML_ASSERT_ENABLED
RELEASE_FLAGS = -O3

vpath %.cpp $(CPP_SOURCE_PATHS)
vpath %.c   $(C_SOURCE_PATHS)

.PHONY: release
.PHONY: debug
.PHONY: clean
.PHONY: flashr
.PHONY: flashd

release: $(OUTPUT_NAME).bin
debug: $(OUTPUT_NAME)_d.bin

clean:
	rm -rf $(OUTDIR)/*

flashr: $(OUTPUT_NAME).bin
	$(OOCD) -f interface/stlink.cfg -f target/stm32l4x.cfg	-c "init" 								\
								-c "reset init" 							\
								-c "program $(OUTPUT_FOLDER_NAME)/$(OUTPUT_NAME).bin 0x08000000 verify"	\
								-c "reset run" 								\
								-c "shutdown"

flashd: $(OUTPUT_NAME)_d.bin
	$(OOCD) -f interface/stlink.cfg -f target/stm32l4x.cfg	-c "init" 								 \
								-c "reset init" 							 \
								-c "program $(OUTPUT_FOLDER_NAME)/$(OUTPUT_NAME)_d.bin 0x08000000 verify"\
								-c "reset run" 								 \
								-c "shutdown"

$(C_OBJECTS_RELEASE): $(OUTDIR_RELEASE)/%_c.o : %.c
	@bash -c 'echo -e $<" \e[01;32m[compiling]\e[0m"'
	mkdir -p $(dir $@)
	$(CC) -c -o $@ $< $(CFLAGS) $(RELEASE_FLAGS)

$(CPP_OBJECTS_RELEASE): $(OUTDIR_RELEASE)/%_cpp.o : %.cpp
	@bash -c 'echo -e $<" \e[01;32m[compiling]\e[0m"'
	mkdir -p $(dir $@)
	$(CXX) -c -o $@ $< $(CPPFLAGS) $(RELEASE_FLAGS)

$(STARTUP_RELEASE): $(STARTUP_FILE)
	@bash -c 'echo -e $<" \e[01;32m[compiling]\e[0m"'
	mkdir -p $(dir $@)
	$(CXX) -c -o $@ $< $(CFLAGS) $(RELEASE_FLAGS)

$(C_OBJECTS_DEBUG): $(OUTDIR_DEBUG)/%_c.o : %.c
	@bash -c 'echo -e $<" \e[01;32m[compiling]\e[0m"'
	mkdir -p $(dir $@)
	$(CC) -c -o $@ $< $(CFLAGS) $(DEBUG_FLAGS)

$(CPP_OBJECTS_DEBUG): $(OUTDIR_DEBUG)/%_cpp.o : %.cpp
	@bash -c 'echo -e $<" \e[01;32m[compiling]\e[0m"'
	mkdir -p $(dir $@)
	$(CXX) -c -o $@ $< $(CPPFLAGS) $(DEBUG_FLAGS)

$(STARTUP_DEBUG) : $(STARTUP_FILE)
	@bash -c 'echo -e $<" \e[ 01;32m[compiling]\e[0m"'
	mkdir -p $(dir $@)
	$(CXX) -c -o $@ $< $(CFLAGS) $(DEBUG_FLAGS)

$(OUTPUT_NAME).elf : $(C_OBJECTS_RELEASE) $(CPP_OBJECTS_RELEASE) $(STARTUP_RELEASE)
	bash -c 'echo -e "$@ \e[01;36m[linking]\e[0m" '
	$(CXX) $^ $(MAIN_LDFLAGS_RELEASE)  -o $(OUTDIR)/$(OUTPUT_NAME).elf

$(OUTPUT_NAME).bin : $(OUTPUT_NAME).elf
	bash -c 'echo -e "$@ \e[01;36m[Final build]\e[0m" '
	$(CP) $(CPFLAGS) $(OUTDIR)/$(OUTPUT_NAME).elf $(OUTDIR)/$(OUTPUT_NAME).bin
	$(OD) $(ODFLAGS) $(OUTDIR)/$(OUTPUT_NAME).elf> $(OUTDIR)/$(OUTPUT_NAME).lst

$(OUTPUT_NAME)_d.elf : $(C_OBJECTS_DEBUG) $(CPP_OBJECTS_DEBUG) $(STARTUP_DEBUG)
	bash -c 'echo -e "$@ \e[01;36m[linking]\e[0m" '
	$(CXX) $^ $(MAIN_LDFLAGS_DEBUG)  -o $(OUTDIR)/$(OUTPUT_NAME)_d.elf

$(OUTPUT_NAME)_d.bin : $(OUTPUT_NAME)_d.elf
	bash -c 'echo -e "$@ \e[01;36m[Final build]\e[0m" '
	$(CP) $(CPFLAGS) $(OUTDIR)/$(OUTPUT_NAME)_d.elf $(OUTDIR)/$(OUTPUT_NAME)_d.bin
	$(OD) $(ODFLAGS) $(OUTDIR)/$(OUTPUT_NAME)_d.elf> $(OUTDIR)/$(OUTPUT_NAME)_d.lst