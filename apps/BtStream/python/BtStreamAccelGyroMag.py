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
   print "  BtStreamAccelGyroMag.py Com5"
   print " or"
   print "  BtStreamAccelGyroMag.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

# send the set sensors command 
   ser.write(struct.pack('BBB', 0x08, 0xE0, 0x00))    # accel, gyro and mag
   wait_for_ack()

# send the set sampling rate command
#   ser.write(struct.pack('BB', 0x05, 0x64))           #  10.24Hz
   ser.write(struct.pack('BB', 0x05, 0x14))           #  51.20Hz
#   ser.write(struct.pack('BB', 0x05, 0x0A))           # 102.40Hz
#   ser.write(struct.pack('BB', 0x05, 0x04))           # 256.00Hz
   wait_for_ack()

   print "TimeStamp: AccelX AccelY AccelZ | GyroX GyroY GyroZ |  MagX  MagY  MagZ"

# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data   
   ddata = ""
   numbytes = 0
   framesize = 21  #i.e. Packet type (1), TimeStamp (2), 3xAccel (3x2), 3xGyro (3x2), 3xMag (3x2)
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)

         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)
         
         packettype = struct.unpack('B', data[0:1])
         (timestamp, accelx, accely, accelz, gyrox, gyroy, gyroz, magx, magy, magz) = struct.unpack('HHHHHHHhhh', data[1:framesize])
         print "    %5d:   %4d   %4d   %4d |  %4d  %4d  %4d |  %4d  %4d  %4d" % (timestamp, accelx, accely, accelz, gyrox, gyroy, gyroz, magx, magy, magz)
   except KeyboardInterrupt:
# send stop streaming command
      ser.write(struct.pack('B', 0x20));
      wait_for_ack()
# close the serial port
      ser.close()
      print
      print "All done"
