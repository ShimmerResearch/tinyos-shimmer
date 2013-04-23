#!/usr/bin/python

#pyserial needs to be installed for this to work: http://pyserial.sourceforge.net/

import sys, serial, struct

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specifiy the serial port of the shimmer you wish to connect to"
   print "example:"
   print "  testGPS.py Com5"
   print " or"
   print "  testGPS.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

# read incoming data   
   ddata = ""
   numbytes = 0
   framesize = 96

   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)

         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         (payloadsize,) = struct.unpack('B', data[0:1])

         if payloadsize == 6:
            (temp,press) = struct.unpack('=hi', data[2:8])
            print "%.1f   %.2f" % (temp/10.0, press/100.0)
         else:
            print str(data[2:])
         
   except KeyboardInterrupt:
# close the serial port
      ser.close()
      print
      print "All done"
