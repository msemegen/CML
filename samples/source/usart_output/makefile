#
#  Name: makefile
#
#  Copyright (c) Mateusz Semegen and contributors. All rights reserved.
#  Licensed under the MIT license. See LICENSE file in the project root for details.
#

ifndef NOSILENT
.SILENT:
endif

PROJECT_NAME := usart_output
ROOT         := $(CURDIR)
CML_ROOT     := $(ROOT)/../../..
LIBRARIES    := $(ROOT)/libraries

MCU_ARCH    := ARM_CORTEX_M4
MCU_FAMILY  := STM32L4
MCU_TYPE    := STM32L452xx
MCU_MODEL   := STM32L452RETx
OUTPUT_NAME := $(PROJECT_NAME)_$(MCU_MODEL)

CPP_SOURCE_PATHS := $(CML_ROOT)/samples/source/usart_output

OUTPUT_FOLDER_NAME := output
OUTDIR         	   := $(ROOT)/$(OUTPUT_FOLDER_NAME)
OUTDIR_DEBUG   	   := $(OUTDIR)/debug
OUTDIR_RELEASE 	   := $(OUTDIR)/release

include $(ROOT)/../modules.mk
include $(ROOT)/../../tc.mk

LD_PATH = $(ROOT)/../../ld

include $(ROOT)/../build.mk