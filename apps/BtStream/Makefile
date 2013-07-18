COMPONENT=BtStreamAppC

#defines
CFLAGS += -DUSE_8MHZ_CRYSTAL
#CFLAGS += -DUSE_230400_BAUD   # allows for higher throughput over Bluetooth
                              # but before using this flag for the first time 
                              # the baud rate of the RN42 module needs to be
                              # changed. Equally it needs to be changed back
                              # to the default if/when this flag is no longer
                              # being used

include $(MAKERULES)

#used to identify shimmer version
ifeq ($(PLATFORM),shimmer2r)
   CFLAGS += -DPWRMUX_UTIL
   CFLAGS += -DSHIMMER_VER=2
else ifeq ($(PLATFORM),shimmer2)
   CFLAGS += -DSHIMMER_VER=1
else ifeq ($(PLATFORM),shimmer)
   CFLAGS += -DSHIMMER_VER=0
endif

#BoilerPlate version
CFLAGS += -DFW_IDENTIFIER=1   # Two byte firmware identifier number
                              # See FirmwareIdentifierList.txt
CFLAGS += -DFW_VER_MAJOR=1    # Major version number: 0-65535U
CFLAGS += -DFW_VER_MINOR=2    # Minor version number: 0-255
CFLAGS += -DFW_VER_REL=0      # Release candidate version number: 0-255
