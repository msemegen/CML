#
#  Name: makefile
#
#  Copyright (c) Mateusz Semegen and contributors. All rights reserved.
#  Licensed under the MIT license. See LICENSE file in the project root for details.
#

ifndef NOSILENT
.SILENT:
endif

PROJECT_NAME := cml_stm32l452xx_minimal
ROOT         := $(CURDIR)
CML_ROOT     := $(ROOT)/../../../..
LIBRARIES    := $(ROOT)/libraries
OUTPUT_NAME  := $(PROJECT_NAME)

MCU_FAMILY := STM32L4
MCU_TYPE   := STM32L412xx

OUTPUT_FOLDER_NAME := output
OUTDIR         	   := $(ROOT)/$(OUTPUT_FOLDER_NAME)
OUTDIR_DEBUG   	   := $(OUTDIR)/debug
OUTDIR_RELEASE 	   := $(OUTDIR)/release

include $(ROOT)/../modules.mk
include $(ROOT)/../../../tc.mk

LD_PATH = $(ROOT)/../

include $(ROOT)/../build.mk