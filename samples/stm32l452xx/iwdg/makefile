ifndef NOSILENT
.SILENT:
endif

PROJECT_NAME := cml_iwdg_sample
ROOT         := $(CURDIR)
LIBRARIES    := $(ROOT)/libraries
OUTPUT_NAME  := $(PROJECT_NAME)

OUTPUT_FOLDER_NAME := output
OUTDIR         	   := $(ROOT)/$(OUTPUT_FOLDER_NAME)
OUTDIR_DEBUG   	   := $(OUTDIR)/debug
OUTDIR_RELEASE 	   := $(OUTDIR)/release

include $(ROOT)/../modules.mk
include $(ROOT)/../../tc.mk

LD_PATH = $(ROOT)/../

include $(ROOT)/../build.mk