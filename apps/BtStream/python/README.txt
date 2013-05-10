This folder contains a number of simple python applications that demonstrate how to interact with a shimmer running the BtStream firmware. See the README.txt and Shimmer.h files in teh BtStream application folder for information on the used commands/responses (along with the other available functionality)

Each of these have been tested using python 2.7.3.

These demonstrate three different methods of communicating with the shimmer: using pyserial (http://pyserial.sourceforge.net/) which is fully cross platform, pybluez on Linux (http://code.google.com/p/pybluez/), and LightBlue on Mac OS (http://lightblue.sourceforge.net/). 
From experience pybluez seems more reliable when making a connection with a shimmer in Linux than pyserial.

The pyserial examples have each been tested in Linux (Ubuntu 10.04), Windows (XP and Win7) and Mac OS X (10.4 and 10.6).

Example applications:
Name                    | Description
------------------------|-------------------------------------------------------------------------------
accelCalRead.py         | Example of reading accelerometer calibration data from shimmer (uses pyserial)
accelCalWrite.py        | Example of writing accelerometer calibration data to shimmer (uses pyserial)
BtStreamAccelGyroMag.py | Example of configuring sensors to be sampled and sampling rate, starting and 
                        | stopping streaming, and parsing the received data (uses pyserial)
BtStreamAccel_Linux.py  | Example of configuring sensors to be sampled and sampling rate, starting and 
                        | stopping streaming, and parsing the received data (uses pybluez)
BtStreamAccel_Mac.py    | Example of configuring sensors to be sampled and sampling rate, starting and 
                        | stopping streaming, and parsing the received data (uses LightBlue)
bufferTest.py           | Example of configuring sensors to be sampled and sampling rate, setting a 
                        | buffer size greater than 1 (the default, starting and stopping streaming, and 
                        | parsing the blocks of received data (uses pyserial)
getBtStreamVersion.py   | Example of reading the firmware version
getShimmerVersion.py    | Example of reading the version of shimmer hardware being used
gsrRangeTest.py         | Example of configuring sensors to be sampled and sampling rate, setting gsr  
                        | range, starting and stopping streaming, and parsing the received data, including
                        | separating the gsr range value from the raw ADC values (uses pyserial)
gyroTest.py             | Example of configuring sensors to be sampled and sampling rate, reading existing
                        | configuration setup, modifying configuration setup, starting and stopping 
                        | streaming, and parsing the received data (uses pyserial)



Using the pyserial examples in Linux:
Requires the BlueZ Bluetooth Bluetooth libraries and tools to be installed. See http://www.bluez.org for details.

Before running these applications the MAC address of the target shimmer must be bound to an rfcomm port. E.g.:
All the commands given here should be entered from the command line (in a terminal window).
1.	Ensure the Bluetooth radio is available by running the “hciconfig” command.
E.g.:
   tiny2@ShimmerLive:~/Desktop$ hciconfig
	hci0:	Type: USB
	BD Address: 00:19:0E:0A:D6:62 ACL MTU: 1021:8 SCO MTU: 64:1
	UP RUNNING PSCAN 
	RX bytes:1013 acl:0 sco:0 events:34 errors:0
	TX bytes:1347 acl:0 sco:0 commands:34 errors:0

2.	Scan for the Shimmer by running “hcitool scan”.
E.g. :
	tiny2@ShimmerLive:~/Desktop$ hcitool scan
Scanning ...
		00:06:66:42:22:BD	RN42-22BD
		00:A0:96:28:DF:E8	FireFly-DFE8
		00:06:66:42:24:18	RN42-2418

3.	To use the shimmer2r named RN42-2418 it must be bound to an rfcomm device. The “rfcomm bind <n>  <MAC_ADDRESS>” command achieves this. The <n> gives the rfcomm device number, which much be different for each Shimmer paired, and the <MAC_ADDRESS> is the Shimmers MAC address, as can be seen from the “hcitool scan” output above. This command normally needs root privileges, so “sudo” is used.
E.g.:  
	tiny2@ShimmerLive:~/Desktop$ sudo rfcomm bind 0 00:06:66:42:24:18
[sudo] password for tiny2:

4.	Running the “rfcomm” command with no arguments shows which Shimmer is bound to which rfcomm device, along with the current connection status.
E.g.:
	tiny2@ShimmerLive:~/Desktop$ rfcomm
rfcomm0: 00:06:66:42:24:18 channel 1 clean 
rfcomm1: 00:A0:96:28:DF:E8 channel 1 clean

5. The shimmer with address 00:06:66:42:24:18 can then be accessed via /dev/rfcomm0
and the shimmer with address 00:A0:96:28:DF:E8 can be accessed via /dev/rfcomm1
