#!/usr/bin/python
import sys, struct, serial

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

   Xoffset = 2048
   Yoffset = 2048
   Zoffset = 2048
   Xsensitivity = 101
   Ysensitivity = 101
   Zsensitivity = 101
   align0 = -1.00
   align1 = 0.00
   align2 = 0.00
   align3 = 0.00
   align4 = -1.00
   align5 = 0.00
   align6 = 0.00
   align7 = 0.00
   align8 = 1.00

   print "Sending following calibration values to shimmer:\n"
   print "Offset Vector (ba) | Sensitivity Matrix (Ka) | Alignment Matrix (Ra)"
   print "              %4d |    %4d       0       0 |" % (Xoffset, Xsensitivity),
   print ' {: .2f}  {: .2f}  {: .2f}'.format(align0, align1, align2)
   print "              %4d |       0    %4d       0 |" % (Yoffset, Ysensitivity),
   print ' {: .2f}  {: .2f}  {: .2f}'.format(align3, align4, align5)
   print "              %4d |       0       0    %4d |" % (Zoffset, Zsensitivity),
   print ' {: .2f}  {: .2f}  {: .2f}'.format(align6, align7, align8)
   print


# send set accel calibration command plus the calibration values
   data = struct.pack('B', 0x11) + struct.pack('>hhhhhhbbbbbbbbb', Xoffset,Yoffset,Zoffset,Xsensitivity,Ysensitivity,Zsensitivity,int(align0*100),int(align1*100),int(align2*100),int(align3*100),int(align4*100),int(align5*100),int(align6*100),int(align7*100),int(align8*100)) #two calls to struct.pack() required due to alignment issues
   print "Raw packet sent to shimmer:",
   print ",".join("0x{:02x}".format(ord(c)) for c in data)
   print

   ser.write(data)

# read the acknowledgement
   wait_for_ack()
   print "Acknowledgement received for set accel calibration command"
   print

   ser.close();
