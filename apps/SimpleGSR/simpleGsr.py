#!/usr/bin/python
import serial
import sys, struct, array

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specifiy the serial port of the shimmer you wish to connect to"
   print "example:"
   print "  simpleGsr.py COM5"
   print "or"
   print "  simpleGsr.py /dev/rfcomm0"
   print
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

   ddata = ""
   numbytes = 0

   framesize = 80 

   num = 0
   
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)

         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         for i in range(0,framesize,4):
            gsr = struct.unpack('I', data[i:i+4])
            print "%10d" % gsr,
            sys.stdout.flush()
            num += 1
            if num == 5:
               print
               num = 0
   except KeyboardInterrupt:
      ser.close()
      print
      print "All done"
