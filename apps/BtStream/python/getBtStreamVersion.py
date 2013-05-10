#!/usr/bin/python
import sys, struct, array, time, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff) 
   while ddata != ack:
      ddata = ser.read(1)
   return

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specifiy the serial port of the shimmer you wish to connect to"
   print "example:"
   print "  getBtStreamVersion.py Com5"
   print " or"
   print "  getBtStreamVersion.py /dev/rfcomm0"

   print
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

# send the get shimmer version command
   ser.write(struct.pack('B', 0x2E))
   wait_for_ack()
   print "Ack received for get Boilerplate version command"
   
   ddata = ""
   bpversionresponse = struct.pack('B', 0x2F)
   while ddata != bpversionresponse:
      ddata = ser.read(1)
    
   ddata = ""
   numbytes = 0
   framesize = 6
   while numbytes < framesize:
      ddata += ser.read(framesize)
      numbytes = len(ddata)

   data = ddata[0:framesize]
   (identifier, majorver, minorver, releasever) = struct.unpack('HHBB', data);
   print "Firmware identifier: %d" % (identifier) 
   print "BtStream version: %d.%d.%d" % (majorver, minorver, releasever)
   ser.close()
