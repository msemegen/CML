#
#  Name: makefile
#
#  Copyright (c) Mateusz Semegen and contributors. All rights reserved.
#  Licensed under the MIT license. See LICENSE file in the project root for details.
#

ifndef NOSILENT
.SILENT:
endif

PROJECT_NAME := blinky
ROOT         := $(CURDIR)
CML_ROOT     := $(ROOT)/../../../..
LIBRARIES    := $(ROOT)/libraries

MCU_ARCH    := M4
MCU_FAMILY  := STM32L4
MCU_TYPE    := STM32L432xx
MCU_MODEL   := STM32L432KC
OUTPUT_NAME := $(PROJECT_NAME)_$(MCU_MODEL)

CPP_SOURCE_PATHS := $(CML_ROOT)/samples/source/Nucleo-32/blinky

OUTPUT_FOLDER_NAME := output
OUTDIR         	   := $(ROOT)/$(OUTPUT_FOLDER_NAME)
OUTDIR_DEBUG   	   := $(OUTDIR)/debug
OUTDIR_RELEASE 	   := $(OUTDIR)/release

include $(ROOT)/../modules.mk
include $(ROOT)/../../../tc.mk

LD_PATH = $(ROOT)/../../../ld

include $(ROOT)/../build.mk