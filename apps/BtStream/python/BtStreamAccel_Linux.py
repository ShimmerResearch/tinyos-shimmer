#!/usr/bin/python
import sys, struct, array
from bluetooth import *

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff) 
   while ddata != ack:
      ddata = sock.recv(1)
   return

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specifiy the MAC address of the shimmer you wish to connect to"
   print "example:"
   print "  BtStreamAccel_Linux.py 00:06:66:42:24:18"
   print
else:
   port = 1;
   host = sys.argv[1]

   sock = BluetoothSocket( RFCOMM )
   sock.connect((host, port))

# send the set sensors command 
   sock.send(struct.pack('BBB', 0x08, 0x80, 0x00))    # accel only
   wait_for_ack()

# send the set sampling rate command
#   sock.send(struct.pack('BB', 0x05, 0x14))           # 51.2Hz
   sock.send(struct.pack('BB', 0x05, 0xCC))           # 5Hz
   wait_for_ack()

# send start streaming command
   sock.send(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data   
   ddata = ""
   numbytes = 0
   framesize = 9  #default. i.e. Packet type (1), TimeStamp (2), 3xAccel (3x2)
   try:
      while True:
         while numbytes < framesize:
            ddata += sock.recv(framesize)
            numbytes = len(ddata)

         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)
         
         packettype = struct.unpack('B', data[0:1])
         (timestamp, accelx, accely, accelz) = struct.unpack('HHHH', data[1:framesize])
         print "%05d: %04d %04d %04d" % (timestamp, accelx, accely, accelz)
   except KeyboardInterrupt:
# send stop streaming command
      sock.send(struct.pack('B', 0x20));
      wait_for_ack()
# close the socket
      sock.close()
      print
      print "All done"
