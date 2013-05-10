#!/usr/bin/python
import sys, struct, array, serial

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
   print "  gsrRangeTest.py Com5"
   print " or"
   print "  gsrRangeTest.py /dev/rfcomm0"   
   print
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

# send the set sensors command 
   ser.write(struct.pack('BBB', 0x08, 0x04, 0x00))    # gsr only
   wait_for_ack()

# send the set sampling rate command
   ser.write(struct.pack('BB', 0x05, 0x64))           # 10.24Hz
   wait_for_ack()

# send the set gsr range command
   ser.write(struct.pack('BB', 0x21, 0x04))           # autorange 
#   ser.write(struct.pack('BB', 0x21, 0x03))
   wait_for_ack()

# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data   
   ddata = ""
   numbytes = 0
   framesize = 5  #default. i.e. Packet type (1), TimeStamp (2), GSR (2)
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)

         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)
         
         packettype = struct.unpack('B', data[0:1])
         (timestamp, gsr) = struct.unpack('HH', data[1:framesize])
         print "%05d: %04d\t%02d" % (timestamp, (gsr&0x3FFF), ((gsr&0xC000)>>14))
   except KeyboardInterrupt:
# send stop streaming command
      ser.write(struct.pack('B', 0x20));
      wait_for_ack()
# close the serial port
      ser.close()
      print
      print "All done"
