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
   print "  bufferTest.py Com5"
   print " or"
   print "  bufferTest.py /dev/rfcomm0"
   print
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

# send the set sensors command 
   ser.write(struct.pack('BBB', 0x08, 0x80, 0x00))    # accel only
   wait_for_ack()

# send the set sampling rate command
   ser.write(struct.pack('BB', 0x05, 0x14))           # 51.2Hz
#   ser.write(struct.pack('BB', 0x05, 0xC8))           # 5.12Hz
   wait_for_ack()

   buffersize = 31   #max allowable size in this configuration 
# send the set buffer size command
   ser.write(struct.pack('BB', 0x34, buffersize))
   wait_for_ack()

# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data   
   ddata = ""
   numbytes = 0
   samplesize = 8                         #TimeStamp (2), 3xAccel (3x2)
   framesize = 1+(samplesize*buffersize)  #Packet type (1), ((TimeStamp (2), 3xAccel (3x2)) x buffersize)
   lasttimestamp = 0
   print"Timestamp\tdiff\trate\t|| Accel X Accel Y Accel Z"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)

         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)
         
         packettype = struct.unpack('B', data[0:1])
         for i in range(buffersize):
            (timestamp, accelx, accely, accelz) = struct.unpack('HHHH', data[(1+(samplesize*i)):(9+(samplesize*i))])
            if timestamp < lasttimestamp:
               diff = timestamp+ 0xFFFE - lasttimestamp
            else:
               diff = timestamp - lasttimestamp
            rate = float(32768) / diff
            print "%05d\t\t%05d\t%0.3f\t||   %04d    %04d    %04d" % (timestamp, diff, rate, accelx, accely, accelz)
            lasttimestamp = timestamp
   except KeyboardInterrupt:
# send stop streaming command
      ser.write(struct.pack('B', 0x20));
      wait_for_ack()
# close the serial port
      ser.close()
      print
      print "All done"
