COMPONENT=SDLogAppC

# this tells the sd driver to use proper clock calibration for spi bus during card init
PFLAGS += -DSMCLK_4MHZ


include $(MAKERULES)

CFLAGS += -DFATFS_REALTIME=0 # for lower ROM requirement

#used to identify shimmer version
ifeq ($(PLATFORM),shimmer2r)
   CFLAGS += -DPWRMUX_UTIL
   CFLAGS += -DSHIMMER_VER=2
else ifeq ($(PLATFORM), shimmer2)
   CFLAGS += -DSHIMMER_VER=1
else ifeq ($(PLATFORM), shimmer)
   CFLAGS += -DSHIMMER_VER=0
endif

#BoilerPlate version
CFLAGS += -DBOILERPLATE_VER_TYPE=2 # 2 = SD Log
CFLAGS += -DBOILERPLATE_VER_MAJOR=1
CFLAGS += -DBOILERPLATE_VER_MINOR=7
CFLAGS += -DBOILERPLATE_VER_REL=2


