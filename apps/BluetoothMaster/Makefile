COMPONENT=BluetoothMasterTestAppC
TINYOS_ROOT_DIR?=../..
include $(TINYOS_ROOT_DIR)/Makefile.include

ifndef BT_SLAVE
$(error You need to provide a Bluetooth Slave Address to connect to so do make BT_SLAVE=0011f6057c35 shimmer2....where 0011f6057c35 is your Bluetooth device address.)
else
    PFLAGS += -DBT_SLAVE_ADDRESS=\""$(BT_SLAVE)\""
endif
