ifndef NOSILENT
.SILENT:
endif

PROJECT_NAME := cml_exti_controller_sample
ROOT         := $(CURDIR)
CML_ROOT     := $(ROOT)/../../..
LIBRARIES    := $(ROOT)/libraries
OUTPUT_NAME  := $(PROJECT_NAME)

C_SOURCE_PATHS := $(ROOT)/../

OUTPUT_FOLDER_NAME := output
OUTDIR         	   := $(ROOT)/$(OUTPUT_FOLDER_NAME)
OUTDIR_DEBUG   	   := $(OUTDIR)/debug
OUTDIR_RELEASE 	   := $(OUTDIR)/release

include $(ROOT)/../modules.mk
include $(ROOT)/../../tc.mk

LD_PATH = $(ROOT)/../

include $(ROOT)/../build.mk