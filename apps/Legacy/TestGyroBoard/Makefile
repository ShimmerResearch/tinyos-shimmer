# When building consider using the following options:
#
CURRENT_TIME=$(shell python ./timeSec.py) #Set the time on the clock to current time. no ntp means you need a good starting point

COMPONENT=TestGyroBoardC
TINYOS_ROOT_DIR?=../..
include $(TINYOS_ROOT_DIR)/Makefile.include

ifdef CURRENT_TIME
PFLAGS += -DCURRENT_TIME=$(CURRENT_TIME)
endif

