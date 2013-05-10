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
   print "  getShimmerVersion.py Com5"
   print " or"
   print "  getShimmerVersion.py /dev/rfcomm0"
   print
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

# send the get shimmer version command
   ser.write(struct.pack('B', 0x24))
   wait_for_ack()
   print "Ack received for get shimmer version command"
   
   ddata = ""
   shimmerversionresponse = struct.pack('B', 0x25)
   while ddata != shimmerversionresponse:
      ddata = ser.read(1)
   shimmerversion = struct.unpack('B', ser.read(1))
   if shimmerversion[0] == 0:
      print "Shimmer Version: shimmer 1.3"
   elif shimmerversion[0] == 1:
      print "Shimmer Version: shimmer 2"
   elif shimmerversion[0] == 2:
      print "Shimmer Version: shimmer 2r"
   ser.close()
