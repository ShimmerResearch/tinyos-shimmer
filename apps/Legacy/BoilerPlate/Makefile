# NB: As of May 2013 BoilerPlate is deprecated and replaced by the BtStream application

COMPONENT=BoilerPlateAppC
TINYOS_ROOT_DIR?=../..
include $(TINYOS_ROOT_DIR)/Makefile.include

#defines
CFLAGS += -DUSE_8MHZ_CRYSTAL

ifeq ($(PLATFORM),shimmer2r)
   CFLAGS += -DPWRMUX_UTIL
   CFLAGS += -DSHIMMER_VER=2
else ifeq ($(PLATFORM),shimmer2)
   CFLAGS += -DSHIMMER_VER=1
else ifeq ($(PLATFORM),shimmer)
   CFLAGS += -DSHIMMER_VER=0
endif
