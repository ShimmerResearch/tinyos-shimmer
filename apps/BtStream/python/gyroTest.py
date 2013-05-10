#!/usr/bin/python
import sys, struct, array, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff) 
   while ddata != ack:
      ddata = ser.read(1)
   return

def get_config_byte0():
# send get config byte 0 command
   ser.write(struct.pack('B', 0x10))
   wait_for_ack()
# read the response
   ddata = ""
   configsetupbyte0response = struct.pack('B', 0x0F)
   while ddata != configsetupbyte0response:
      ddata = ser.read(1)
   (configbyte0,) = struct.unpack('B', ser.read(1))
   print "Config byte0: 5VREG =", 
   if (configbyte0&0x80) != 0:
      print "On, PMUX =",
   else: 
      print "Off, PMUX =",
   if (configbyte0&0x40) != 0:
      print "On, GyroTempVref =",
   else:
      print "Off, GyroTempVref =",
   if (configbyte0&0x20) != 0:
      print "On"
   else:
      print "Off"
   return

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specifiy the serial port of the shimmer you wish to connect to"
   print "example:"
   print "  gyroTest.py Com5"
   print " or"
   print "  gyroTest.py /dev/rfcomm0"
   print
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

# send the set sensors command 
   ser.write(struct.pack('BBB', 0x08, 0x40, 0x00))    # gyro only
   wait_for_ack()

# send the set sampling rate command
   ser.write(struct.pack('BB', 0x05, 0x64))           # 10.24Hz
   wait_for_ack()
   
# get config byte 0
   get_config_byte0()

# set gyro temp/vref using SET_GYRO_TEMP_VREF_COMMAND
   print "\nSet gyro temp/vref using SET_GYRO_TEMP_VREF_COMMAND\n"
   ser.write(struct.pack('BB', 0x33, 0x01))
   wait_for_ack()

# unset gyro temp/vref using SET_GYRO_TEMP_VREF_COMMAND
#   print "\nUnset gyro temp/vref using SET_GYRO_TEMP_VREF_COMMAND\n"
#   ser.write(struct.pack('BB', 0x33, 0x00))
#   wait_for_ack()

# set gyro temp/vref using SET_CONFIG_SETUP_BYTE0_COMMAND
#   print "\nSet gyro temp/vref using SET_CONFIG_SETUP_BYTE0_COMMAND\n"
#   ser.write(struct.pack('BB', 0x0E, 0x20))
#   wait_for_ack()

# Unset gyro temp/vref using SET_CONFIG_SETUP_BYTE0_COMMAND
#   print "\nUnset gyro temp/vref using SET_CONFIG_SETUP_BYTE0_COMMAND\n"
#   ser.write(struct.pack('BB', 0x0E, 0x00))
#   wait_for_ack()

# get config byte 0
   get_config_byte0()

# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()
# read incoming data   
   ddata = ""
   numbytes = 0
   framesize = 9  #default. i.e. Packet type (1), TimeStamp (2), 3xAccel (3x2)
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)

         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)
         
         packettype = struct.unpack('B', data[0:1])
         (timestamp, gyrox, gyroy, gyroz) = struct.unpack('HHHH', data[1:framesize])
         print "%05d: %04d %04d %04d" % (timestamp, gyrox, gyroy, gyroz)
   except KeyboardInterrupt:
# send stop streaming command
      ser.write(struct.pack('B', 0x20));
      wait_for_ack()
# close the serial port
      ser.close()
      print
      print "All done"
