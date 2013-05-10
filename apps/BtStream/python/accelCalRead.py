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
   print "  accelCalRead.py Com5"
   print " or"
   print "  accelCalRead.py /dev/rfcomm0"
   print
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

# send get accel calibration command
   ser.write(struct.pack('B', 0x13))

# read the acknowledgement
   wait_for_ack()
   print "Acknowledgement received to get accel calibration command"

# wait for calibration response 
   ddata = ""
   response = struct.pack('B', 0x12) 
   while ddata != response:
      ddata = ser.read(1)
   print "Accel calibration response:"

# read incoming data   
   ddata = ""
   numbytes = 0
   framesize = 21 
   
   while numbytes < framesize:
      ddata += ser.read(framesize)
      numbytes = len(ddata)

   data = ddata[0:framesize]
   ddata = ddata[framesize:]
   numbytes = len(ddata)


   print "Raw packet received from shimmer:",
   print ",".join("0x{:02x}".format(ord(c)) for c in data)
   print
 
   (Xoffset, Yoffset, Zoffset, Xsensitivity, Ysensitivity, Zsensitivity, align0, align1, align2, align3, align4, align5, align6, align7, align8) = struct.unpack('>hhhhhhbbbbbbbbb', data);
   print "Offset Vector (ba) | Sensitivity Matrix (Ka) | Alignment Matrix (Ra)"
   print "              %4d |    %4d       0       0 |" % (Xoffset, Xsensitivity),
   print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align0)/100, float(align1)/100, float(align2)/100)
   print "              %4d |       0    %4d       0 |" % (Yoffset, Ysensitivity),
   print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align3)/100, float(align4)/100, float(align5)/100)
   print "              %4d |       0       0    %4d |" % (Zoffset, Zsensitivity),
   print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align6)/100, float(align7)/100, float(align8)/100)
   print

   ser.close()
